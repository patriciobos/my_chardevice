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
static loff_t writeIndex = 0;
static loff_t globalReadIndex = 0;

static unsigned int currentReaders;

static struct timer_list timer_cpy_buffers;

typedef struct readerControl
{
	u8 readerID;
	loff_t readIndex; 
	int readerLock;
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
		readers[i].readerID = 0;
		mb();
		atomic_set(&readers[i].readIndex,0);
		//readers[i].readIndex = 0;
		//spin_lock_init(&readers[i].readerSpinlock);
		mutex_init(&readers[i].readerMutex);
		readers[i].readerLock = 0;
		readers[i].readerBuffer[0] = 0;
	}
}

void copier_deamon (void) {

  volatile int status;
  
  mb(); 
  currentReaders = atomic_read(&users) -1;
  
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

  printk(KERN_INFO "my_chardevice: initialization of the timer\n");

  setup_timer( &timer_cpy_buffers, copier_deamon, 0 );

  //printk( "Starting timer to fire in 800ms (%ld)\n", jiffies );
  
  status = mod_timer( &timer_cpy_buffers, jiffies + msecs_to_jiffies(1000) );
  
  if (status != 0) {
    printk(KERN_ERR "Error in timer initialization in %s, %i\n",__FUNCTION__,__LINE__);
    return status;
  }
  
  return status;
  
}

int deinit_char_timer( void ) {

  int status;

  printk(KERN_INFO "my_chardevice: de-initialization of the timer\n");
  
  status = del_timer( &timer_cpy_buffers );
  
  if (status != 0) {
    printk(KERN_ERR "Could not delete the timer in %s, %i\n",__FUNCTION__,__LINE__);
    return status;
  }

  return status;
}


/*  *************************************************************************************************** */
/*  ************************* read function *********************************************************** */
/*  *************************************************************************************************** */

ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {

 	int status;
 	u8 actualReader;
 	loff_t readIndex;
  	
	//printk(KERN_INFO "my_chardevice: Reading BUFF_B count = %lu offset = %lld\n",count,(*f_pos));
	
  /* Validation of user pointers */
	if (filp == NULL || buf == NULL || f_pos == NULL) {
    printk(KERN_ERR "Invalid pointers in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}
	
	/* Get information from actualReader*/
	actualReader = filp->private_data;
	mb();
	readIndex=atomic_read(&readers[actualReader].readIndex);
	
	printk(KERN_INFO "DEBUG Reader = %i, readIndex = %lld, writeIndex = %lld\n", readers[actualReader].readerID, readers[actualReader].readIndex, writeIndex);  

  /* Validate readIndex is not greater than writeIndex */
  if (readIndex > writeIndex) {
    printk(KERN_ERR "readIndex = %lld is greater than writeIndex = %lld\n", readIndex, writeIndex);
    return -1;
	}  

  /* Validation of the count of chars to read */
	if (count < 0 || count > sizeof(readers[actualReader].readerBuffer) ) {
    printk(KERN_ERR "Count = %lu out of buffers range\n", count);
    return -1;
	}

  /* Something to read*/
  if ( readIndex < writeIndex ) {
	  /* Copy data from kernel to user */
	    
	  if (count > writeIndex - readIndex) {
      count = writeIndex - readIndex;	  
	  }
 	  
 	  spin_lock(&lock_buff_b); 
    memcpy(readers[actualReader].readerBuffer, ptr_BUFF_B, BUFFER_SIZE);
    spin_unlock(&lock_buff_b);
    
		status = copy_to_user(buf, readers[actualReader].readerBuffer+readIndex, count);
		
		if(status != 0) {
		  printk(KERN_ERR "Could not copy to user space in %s, %i\n",__FUNCTION__,__LINE__);
		  return -1;
		}

		mb();
		atomic_set(&readers[actualReader].readIndex, readIndex + count);
  	
  	printk(KERN_INFO "my_chardevice: copied %lu bytes from kernel to user\n", count);
  	//printk(KERN_INFO "DEBUG Reader = %i, readIndex = %lld, writeIndex = %lld\n", readers[actualReader].readerID, readers[actualReader].readIndex, writeIndex);  
    
    return count;
  }
  /* Nothing to read*/
  else if (readIndex == writeIndex) { 
    printk(KERN_INFO "DEBUG Reader = %i, readIndex = %lld, writeIndex = %lld\n", readers[actualReader].readerID, readers[actualReader].readIndex, writeIndex);
    printk(KERN_INFO "DEBUG Nothing to new to read, reader = %i is going to sleep zzzzz", readers[actualReader].readerID);
    mutex_lock(&readers[actualReader].readerMutex);
  }
    return 0; 
}

/*  *************************************************************************************************** */
/*  ************************* write function ********************************************************** */
/*  *************************************************************************************************** */

ssize_t dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {

  int status, overflow_flag = 0;
  
  u8 i;

  ssize_t retvalue=-ENOMEM;
	
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
	
  if (count > BUFFER_SIZE - writeIndex) {
    printk(KERN_INFO "Count = %d exceeds buffer free spece = %d.  Overflow condition detected", count, (BUFFER_SIZE - writeIndex) );
    overflow_flag = 1;
    count = BUFFER_SIZE - writeIndex;
  }

	printk(KERN_INFO "my_chardevice: Writting to BUFF_A: %d chars of %s", count, buf);
	//printk(KERN_INFO "my_chardevice: Count parameter from user space %lld.\n", count);
  //printk(KERN_INFO "my_chardevice: Offset in buffer %lld.\n",(long unsigned int)*f_pos);
	  
 
  if ( (*f_pos) >= BUFFER_SIZE ) {
    printk(KERN_INFO "Buffer is already complete");
    return 0;
  }
  
  //TODO: Revisar si quiero dejar el '\0' al final de cada mensaje o no
  spin_lock(&lock_buff_a);
  status = strncpy_from_user( ptr_BUFF_A + (*f_pos), buf, count );
  spin_unlock(&lock_buff_a);

  if (status < 0) {
    printk(KERN_ERR "Could not copy from user to kernel in %s, %i\n",__FUNCTION__,__LINE__);
    retvalue = -EFAULT;
  }
 
  else {
    printk(KERN_INFO "my_chardevice: DEBUG WR count = %lu offset = %lld\n",count,(*f_pos));
    writeIndex += count;
  
    mb(); 
    currentReaders = atomic_read(&users) - 1;
  
    for (i=0;i<=currentReaders;i++) {
      mutex_unlock(&readers[i].readerMutex);
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
	//spin_lock_init(&semaphoreWrite);
	//spin_lock_init(&reader_lock);

  /* First open. need to initialize things*/
  if( 0 == atomic_read(&users) ){
	
  	printk(KERN_INFO "my_chardevice: Opening device and allocating memory for buffers");
  	
  	/* allocate memory for buffers*/
	  ptr_BUFF_A = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_AUX = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_B = kzalloc(BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	
  	if (ptr_BUFF_A == NULL || ptr_BUFF_AUX == NULL || ptr_BUFF_B == NULL) {
      printk(KERN_ERR "Could not allocate memory for BUFFERs in %s, %i\n",__FUNCTION__,__LINE__);
      return -1;
  	}
    
    /* initialize syncronization spinlocks and mutexs */  	
  	spin_lock_init(&lock_buff_a);
    spin_lock_init(&lock_buff_aux);
    spin_lock_init(&lock_buff_b);
    
    //sema_init(&sema_buff_a, 0);

    //mutex_init(&mutex_buff_a);
    
    init_reader_struct();
    
    /* initialization of the timer that triggers the copy from buff_a to buff_aux */
    init_char_timer();
    
  }  
  /* things already initialized */
  else {
    //printk(KERN_INFO "my_chardevice: reader %i is opening the device\n",atomic_read(&users));
    
    mb() ;
		currentReaderId=atomic_read(&users)-1;
		
		mb() ;
		readers[currentReaderId].readerID = currentReaderId;	
		
		filp->private_data = currentReaderId;
		
		readers[currentReaderId].readerBuffer[0] = '\0';
		
		//driverReaders[readerId-1].readIndex = ATOMIC_INIT(0);
		
		printk("Reader %i logged in with ID %d on filp->private_data\n",atomic_read(&users),filp->private_data);
				
  }
  
  //TODO: check if mb() is really needed here
  mb();
  atomic_inc(&users);
  
	return 0;
}

/*  *************************************************************************************************** */
/*  ************************* close function ********************************************************** */
/*  *************************************************************************************************** */

int dev_close(struct inode *inode, struct file *flip){

  //TODO: Check input parameters?
  
  if(!atomic_dec_and_test(&users)) {
    printk(KERN_INFO "my_chardevice: Closing device, %i users remaining\n",atomic_read(&users));
  }
  
  else {
    printk(KERN_INFO "my_chardevice: Closing device, de-allocating memory\n");

   	kfree(ptr_BUFF_A);
   	kfree(ptr_BUFF_AUX);
   	kfree(ptr_BUFF_B);
   	
   	deinit_char_timer();

  }
  
  return 0;
}



struct file_operations dev_fops = { //Struct File Operations, this module only supports read...
	.open = dev_open,
  .release = dev_close,
	.read = dev_read,
	.write = dev_write,
	.owner = THIS_MODULE,           // Tells who is owner of struct file_operations
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
