/* Copyright (C) 2015, Santiago F. Maudet
 * This file is part of char01 module.
 *
 * char01 module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * char01 module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* !\brief This is a basic example of a char device.
 *         Basics of LKM are described.
 *         This module has a initialized buffer in kernel memory that you
 *         can read from user space.
 *         Usage:
 *         1) COMPILE = make
 *         2) Insert module into kernel (root): insmod char_03.ko
 *         3) Create /dev/char_01 node: mknod /dev/char_03 c [MAYOR_NUMBER] [MINOR_NUMBER]
 *         4) cat /dev/char_01 or use open and read from a C program.
 *
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/errno.h>      /* Standard error numbers definitions */

#define MY_CHAR_MINOR 0       /* Minor number at which devices start */
#define MY_CHAR_N_DEVS 1      /* Number of devices to register */

#define DRVNAME  "my_char"
#define DRVCLASS "my_chardrv"
#define DRVDEVICENAME "my_chardevice"

#define BUFFER_SIZE 32
#define MAX_READERS 10

/* Public global variable definitions*/

struct cdev device_cdev;    //Kernel structure for char devices.

spinlock_t lock_buff_a;
spinlock_t lock_buff_aux;
spinlock_t lock_buff_b;

struct semaphore sema_buff_a;
struct semaphore sema_buff_aux;
struct semaphore sema_buff_b;

/* --------------------------------- */

/* Private global variable definitions */

static dev_t device; /*minor and mayor number of the device*/
static struct class * char_classdev;

static char * ptr_BUFF_A = NULL;
static char * ptr_BUFF_AUX = NULL;
static char * ptr_BUFF_B = NULL;

static atomic_t users=ATOMIC_INIT(0);


//TODO: check writeIndex data type.  there are some comparisons between loff_t and atomic_t?
static atomic_t writeIndex = ATOMIC_INIT(0);;

//static loff_t globalReadIndex = 0;

static unsigned int currentReaders;
static unsigned int overflow_flag = 0;
static unsigned int killThemAll = 0;

static struct timer_list timer_cpy_buffers;

typedef struct readerControl
{
	int readerID;
	loff_t readIndex; 
//	int readerLock;
//	spinlock_t readerSpinlock;
  struct mutex readerMutex;
	char readerBuffer[BUFFER_SIZE];
} readerControl_t;

readerControl_t readers[MAX_READERS];

/* ----------------------------------- */

void init_reader_struct(void)
{
	u8 i = 0;
	
	for(i=0; i<MAX_READERS; i++)
	{	
		readers[i].readerID = -1;
		readers[i].readIndex = 0;
		//mb();
		//atomic_set(&readers[i].readIndex,0);
		//readers[i].readIndex = 0;
		//spin_lock_init(&readers[i].readerSpinlock);
		mutex_init(&readers[i].readerMutex);
		mutex_lock(&readers[i].readerMutex);
		//readers[i].readerLock = 0;
		readers[i].readerBuffer[0] = 0;
	}
}

void copier_deamon (void) {

  volatile int status;
  
  mb(); 
  currentReaders = atomic_read(&users) - 1;
  
  //printk(KERN_INFO "my_chardevice: copier_deamon! readers: %i\n", currentReaders);
   
  spin_lock(&lock_buff_a);
  memcpy(ptr_BUFF_AUX, ptr_BUFF_A,BUFFER_SIZE);
  spin_unlock(&lock_buff_a);


  spin_lock(&lock_buff_aux);
  memcpy(ptr_BUFF_B, ptr_BUFF_AUX,BUFFER_SIZE);
  spin_unlock(&lock_buff_aux);
 
  /* re-arm the timer before return */
  status = mod_timer( &timer_cpy_buffers, jiffies + msecs_to_jiffies(1000) );
  
  if (status != 0) {
    printk(KERN_ERR "Error in timer re-initialization in %s, %i\n",__FUNCTION__,__LINE__);
  }
  
}

int init_char_timer( void ) {

  int status;

  printk(KERN_INFO "my_chardevice: initializing the timer\n");

  setup_timer( &timer_cpy_buffers, (void *)copier_deamon, 0 );

  //printk( "Starting timer to fire in 800ms (%ld)\n", jiffies );
  
  status = mod_timer( &timer_cpy_buffers, jiffies + msecs_to_jiffies(1000) );
  
  if (status != 0) 
    printk(KERN_ERR "Error in timer initialization in %s, %i\n",__FUNCTION__,__LINE__);
  
  return status;
  
}

void deinit_char_timer( void ) {

  int status;

  //printk(KERN_INFO "my_chardevice: de-initialization of the timer\n");
  
  status = del_timer( &timer_cpy_buffers );
  
  if (status != 0) 
    printk(KERN_ERR "The timer is still in use...\n");

  return;

}


/*  *************************************************************************************************** */
/*  ************************* read function *********************************************************** */
/*  *************************************************************************************************** */

ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {

 	int status, readIndexLocal, writeIndexLocal;
 	
 	u8 actualReader;
 	  
  /* Get information from actualReader*/
	actualReader = (unsigned short int) filp->private_data;

	readIndexLocal = readers[actualReader].readIndex;
	
	mb();
	writeIndexLocal = atomic_read(&writeIndex);
  
  printk(KERN_INFO "\nDEBUG reader %i: readIndex = %d, writeIndex = %d\n",actualReader, readIndexLocal, writeIndexLocal);  
  		
  /* Validation of user pointers */
	if (filp == NULL || buf == NULL || f_pos == NULL) {
    printk(KERN_ERR "Invalid pointers in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}	

  /* Validate readIndex is not greater than writeIndex */
  if (readIndexLocal > writeIndexLocal) {
    printk(KERN_ERR "readIndex = %d is greater than writeIndex = %d\n", readIndexLocal, writeIndexLocal);
    return -1;
	}  

  /* Validation of the count of chars to read */
	if (count < 0 || count > sizeof(readers[actualReader].readerBuffer) ) {
    printk(KERN_ERR "Count = %lu out of buffers range\n", count);
    return -1;
	}
  
  //TODO: check overflow condition and if writer hasn't exit

  if (readIndexLocal == writeIndexLocal) { 
    printk(KERN_INFO "DEBUG reader %i: Nothing new to read, going to sleep\n", actualReader); 
    mutex_lock(&readers[actualReader].readerMutex);
  }
  
  /* Something to read*/
  if ( readIndexLocal < writeIndexLocal ) {
	  /* Copy data from kernel to user */
	    
	  if (count > writeIndexLocal - readIndexLocal) {
      count = writeIndexLocal - readIndexLocal;	  
	  }
 	  
 	  spin_lock(&lock_buff_b); 
    memcpy(readers[actualReader].readerBuffer, ptr_BUFF_B, BUFFER_SIZE);
    spin_unlock(&lock_buff_b);
    
		status = copy_to_user(buf, (readers[actualReader].readerBuffer)+readIndexLocal, count);
		
		if(status != 0) {
		  printk(KERN_ERR "Could not copy to user space in %s, %i\n",__FUNCTION__,__LINE__);
		  return -1;
		}

		//mb();
		//atomic_set(&readers[actualReader].readIndex, readIndexLocal + count);
  	
  	readers[actualReader].readIndex = readIndexLocal + count;
  	
  	printk(KERN_INFO "DEBUG reader %i: copied %lu bytes from kernel to user\n",actualReader, count);
    
    return count;
  }
  /* Nothing to read*/
  else if (readIndexLocal == writeIndexLocal) { 
        
    if (killThemAll == 1) {
      
      printk(KERN_INFO "DEBUG reader %i: Writer is gone, I will exit too\n", actualReader);
      return -1;
    }
    
  }
    return 0; 
}

/*  *************************************************************************************************** */
/*  ************************* write function ********************************************************** */
/*  *************************************************************************************************** */

ssize_t dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {

  ssize_t retvalue; 
  
  int status, writeIndexLocal;
  u8 i;
  
  mb();
	writeIndexLocal = atomic_read(&writeIndex);
	
  if (filp->private_data != (void *)MAX_READERS) {
    printk(KERN_ERR "forcing ID of writer to MAX_READERS\n");
    filp->private_data = (void *)MAX_READERS;
    }
	
  //TODO: Check input parameters?

  /* Validation of user pointers */
	if (filp == NULL || buf == NULL || f_pos == NULL) {
    printk(KERN_ERR "Invalid pointers in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}
  
  //TODO: check condition of count > BUF_SIZE superposition with overflow
  
  /* Validation of count to write */
	if (count < 0 ) {
    printk(KERN_ERR "Invalid count of chars to write in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}
	
  if (count > BUFFER_SIZE - writeIndexLocal) {
    printk(KERN_INFO "Count = %lu exceeds buffer free space = %d.  Overflow condition detected", count, (BUFFER_SIZE - writeIndexLocal) );
    overflow_flag = 1;
    count = BUFFER_SIZE - writeIndexLocal - 1;
  }

	printk(KERN_INFO "\nDEBUG writer: Writting to BUFF_A: %s", buf );
	printk(KERN_INFO "DEBUG writer: writeIndex = %d, chars to write = %lu, free space = %d\n", writeIndexLocal, count, BUFFER_SIZE - writeIndexLocal );  
 
  if ( writeIndexLocal >= BUFFER_SIZE ) {
    printk(KERN_INFO "Buffer is already complete");
    return 0;
  }
  
  spin_lock(&lock_buff_a);
  status = strncpy_from_user( ptr_BUFF_A + writeIndexLocal, buf, count );
  spin_unlock(&lock_buff_a);

  if (status < 0) {
    printk(KERN_ERR "Could not copy from user to kernel in %s, %i\n",__FUNCTION__,__LINE__);
    retvalue = -EFAULT;
  }
 
  else {
    //printk(KERN_INFO "DEBUG writer: writeIndex = %d\n", writeIndexLocal);
    //writeIndexLocal += count;
  
    if (overflow_flag == 1) {
      *(ptr_BUFF_A + writeIndexLocal + count + 1) = '\0';
      count++;
             	
    }
    
    mb(); 
    atomic_set(&writeIndex, writeIndexLocal + count);
      
    for (i=0;i<atomic_read(&users)-1;i++) {
      mutex_unlock(&readers[i].readerMutex);
      printk(KERN_INFO "DEBUG writer: unlocked mutex for reader %i \n", i);
    }
    
    retvalue = count;    
  } 
  
    
  return retvalue;
}	


/*  *************************************************************************************************** */
/*  ************************* open function *********************************************************** */
/*  *************************************************************************************************** */

int dev_open(struct inode *inode, struct file *filp) {

	//TODO: Check input parameters?
	
	int currentReaderId=0;
	
  /* First open from writer.  Need to initialize things*/
  if( 0 == atomic_read(&users) ){
	
  	printk(KERN_INFO "my_chardevice: Opening device and allocating memory for buffers");
  	
  	filp->private_data = (void *)MAX_READERS;  // reader are in range 0:MAX_READERS-1 so writer ID is MAX_READERS
  	
  	atomic_set(&writeIndex,0);
  	
  	overflow_flag = 0;  // signal to indicate that the string copied from user space exceeds writer's buffer size
  	killThemAll = 0;    // signal to terminate of readers intances
  	
  	/* allocate memory for buffers*/
	  ptr_BUFF_A = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_AUX = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_B = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	
  	if (ptr_BUFF_A == NULL || ptr_BUFF_AUX == NULL || ptr_BUFF_B == NULL) {
      printk(KERN_ERR "Could not allocate memory for BUFFERs in %s, %i\n",__FUNCTION__,__LINE__);
      return -1;
  	}
    
    /* initialize spinlocks */  	
  	spin_lock_init(&lock_buff_a);
    spin_lock_init(&lock_buff_aux);
    spin_lock_init(&lock_buff_b);
    
    /* initialize reader's control struct*/
    init_reader_struct();
    
    /* initialize the timer that triggers the copy of buffers*/
    init_char_timer();
    
  }  
  /* Open from reader. Things already initialized */
  else {
        
    mb() ;
		currentReaderId=atomic_read(&users) - 1;
		
		readers[currentReaderId].readerID = currentReaderId;	
		
		filp->private_data = (void *) currentReaderId;
		
		readers[currentReaderId].readerBuffer[0] = '\0';
				
		printk("\nDEBUG: Reader %ld: logged in\n",(long) filp->private_data);
				
  }
  
  atomic_inc(&users);
  
	return 0;
}

/*  *************************************************************************************************** */
/*  ************************* close function ********************************************************** */
/*  *************************************************************************************************** */

int dev_close(struct inode *inode, struct file *filp){

  //TODO: Check input parameters?
  
  u8 i;
    
  if (filp->private_data == (void *)MAX_READERS) {
    printk(KERN_INFO "DEBUG: writer is exiting, sending signal to reader to exit too\n");
    killThemAll = 1;    // Signal to close all readers instances
    
    mb(); 
 
    for (i=0;i<atomic_read(&users)-1;i++) {
      mutex_unlock(&readers[i].readerMutex);
      printk(KERN_INFO "DEBUG: mutex_unlock so reader %i can exit\n", i);
    }
  }
  
  
  if(!atomic_dec_and_test(&users)) {
    printk(KERN_INFO "my_chardevice: Closing device, %i users remaining\n",atomic_read(&users));
  }
  
  else {
    printk(KERN_INFO "my_chardevice: Closing device, de-allocating memory\n");

   	kfree(ptr_BUFF_A);
   	kfree(ptr_BUFF_AUX);
   	kfree(ptr_BUFF_B);
   	
   	//deinit_char_timer();
   	del_timer_sync(&timer_cpy_buffers);


  }
  
  return 0;
}

/*  *************************************************************************************************** */
/*  ************************* File operations ********************************************************* */
/*  *************************************************************************************************** */

struct file_operations dev_fops = { 
	.open = dev_open,
  .release = dev_close,
	.read = dev_read,
	.write = dev_write,
	.owner = THIS_MODULE,           
};


// Init Function //
static int __init dev_init(void)
{
  int status;  // Aux Variable.

  printk(KERN_INFO "my_chardevice: Loading module\n"); //Kernel Info

  // Dynamic Allocation of MAJOR Number for char_02 Module. 
  status = alloc_chrdev_region(&device,MY_CHAR_MINOR,MY_CHAR_N_DEVS, DRVNAME);

  //Can't get MAJOR Number, return...
  if(status < 0){
    printk(KERN_WARNING "my_chardevice: could not get major number\n");
    return status;
  }

  char_classdev = class_create(THIS_MODULE,DRVCLASS);

  device_create(char_classdev, NULL, device, NULL, DRVDEVICENAME);

  // Initialize struct cdev "device_cdev" with struct file_operations "dev_fops"
  cdev_init(&device_cdev, &dev_fops);

  // Add device to kernel.
  status = cdev_add(&device_cdev, device,MY_CHAR_N_DEVS);
  
  if(status < 0){
    printk(KERN_WARNING "my_chardevice: device could not be registered in the kernel\n");
    return status;
  }
  printk(KERN_INFO "my_chardevice: correct registration of device in the kernel\n");
  return status;
}

// Exit Function //
static void __exit dev_exit(void)
{
  printk(KERN_INFO "my_chardevice: unloading Module from kernel\n");

	//chequear el orden

	//Remove device form kernel.
  cdev_del(&device_cdev); 

	//Device Destroy
  device_destroy(char_classdev, device);

	//Class_destroy()
  class_destroy(char_classdev);

	//Release MAJOR and MINIOR Numbers.
  unregister_chrdev_region(device,MY_CHAR_N_DEVS);
}

module_init(dev_init); //Init Macro loaded with init function.
module_exit(dev_exit); //Exit Macro loaded with exit function.

MODULE_AUTHOR("Patricio Bos, MSE @ FI.UBA");
MODULE_LICENSE("GPL");
