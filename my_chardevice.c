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

//#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/errno.h>      /* Standard error numbers definitions */

#define MY_CHAR_MINOR 0       /* Minor number at which devices start */
#define MY_CHAR_N_DEVS 1      /* Number of devices to register */

#define DRVNAME  "my_char"
#define DRVCLASS "my_chardrv"
#define DRVDEVICENAME "my_chardevice"

#define CHAR_BUFFER_SIZE 32

/* Public global variable definitions*/

struct cdev device_cdev;    //Kernel structure for char devices.

spinlock_t lock_buffer;
spinlock_t lock_buffer_aux;

struct semaphore sema_buff_a;
struct semaphore sema_buff_aux;

//mutex_t mutex_buff_a;

/* --------------------------------- */

/* Private global variable definitions */

static dev_t device; /*minor and mayor number of the device*/
static struct class * char_classdev;

static char * ptr_BUFF_A = NULL;
static char * ptr_BUFF_AUX = NULL;
static char * ptr_BUFF_B = NULL;
static atomic_t users=ATOMIC_INIT(0);

static struct timer_list timer_cpy_buffers;

static loff_t write_index = 0;

static int current_readers;

/* ----------------------------------- */

void copier_deamon (void) {

  volatile int status;
   
  current_readers = atomic_read(&users) -1;
  
  //printk(KERN_INFO "my_chardevice: copier_deamon! readers: %i\n", current_readers);
   
  spin_lock(&lock_buffer);
  memcpy(ptr_BUFF_AUX, ptr_BUFF_A,CHAR_BUFFER_SIZE);
  spin_unlock(&lock_buffer);


  spin_lock(&lock_buffer_aux);
  memcpy(ptr_BUFF_B, ptr_BUFF_AUX,CHAR_BUFFER_SIZE);
  spin_unlock(&lock_buffer_aux);
 
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
    printk(KERN_ERR "Error in timer inizialization in %s, %i\n",__FUNCTION__,__LINE__);
    return status;
  }
  
  return status;
  
}

int deinit_char_timer( void ) {

  int status;

  printk(KERN_INFO "my_chardevice: de-inizialization of the timer\n");
  
  status = del_timer( &timer_cpy_buffers );
  
  if (status != 0) {
    printk(KERN_ERR "Could not delete the timer in %s, %i\n",__FUNCTION__,__LINE__);
    return status;
  }

  return status;
}

//open function
int dev_open(struct inode *inode, struct file *flip) {

	//TODO: Check input parameters?

  /* First open. need to initialize things*/
  if( 0 == atomic_read(&users) ){
	
  	printk(KERN_INFO "my_chardevice: Opening device and allocating memory for buffers");
  	
  	/* allocate memory for buffers*/
	  ptr_BUFF_A = kzalloc(CHAR_BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_AUX = kzalloc(CHAR_BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	  ptr_BUFF_B = kzalloc(CHAR_BUFFER_SIZE*sizeof(char),GFP_KERNEL);
	
  	if (ptr_BUFF_A == NULL || ptr_BUFF_AUX == NULL || ptr_BUFF_B == NULL) {
      printk(KERN_ERR "Could not allocate memory for BUFFERs in %s, %i\n",__FUNCTION__,__LINE__);
      return -1;
  	}
    
    /* initialize syncronization spinlocks and mutex */  	
  	spin_lock_init(&lock_buffer);
    spin_lock_init(&lock_buffer_aux);
    
    //sema_init(&sema_buff_a, 0);

    //mutex_init(&mutex_buff_a);
    
    /* initialization of the timer that triggers the copy from buff_a to buff_aux */
    init_char_timer();
    
  }  
  /* things already initialized */
  else {
    printk(KERN_INFO "my_chardevice: opening device from reader side %i\n",atomic_read(&users));
  }
  
  //TODO: memory barriers needed?
  atomic_inc(&users);
  
	return 0;
}

//close function
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

// Read function //
ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {

 	int status;
  	
	//printk(KERN_INFO "my_chardevice: Reading BUFF_B count = %lu offset = %lld\n",count,(*f_pos));
	
  /* Validation of user pointers */
	if (filp == NULL || buf == NULL || f_pos == NULL) {
    printk(KERN_ERR "Invalid pointers in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}

  /* Validation of the reader offset */
  if ( (* f_pos) < 0 || (* f_pos) > CHAR_BUFFER_SIZE - 1 ) {
    printk(KERN_INFO "Offset value out of range received from user in %s, %i\n",__FUNCTION__,__LINE__);    
    return -1;
  }

  /* Validate read index f_pos is not greater than write_index */
  if ((*f_pos) > write_index) {
    printk(KERN_ERR "read_index = %lld is greater than write_index = %lld\n", *f_pos, write_index);
    return -1;
	}  

  /* Validation of the count of chars to read */
	if (count < 0 || count > (CHAR_BUFFER_SIZE - (*f_pos) ) ) {
    //printk(KERN_ERR "%lu is an invalid number of chars to read\n", count);
    count = CHAR_BUFFER_SIZE  - (*f_pos);
	}

  
  /* Something new since last reading */
	//if ( *(ptr_BUFF_B + (*f_pos) ) != '\0') {
	

  if ( (*f_pos) < write_index ) {
	  /* Copy data from kernel to user */
	  
	  /* user asked for more or exactly the number of available chars         
	   * count should be only de difference between de write and read indexes.
	   * +1 to include de '\0' string delimiter */
	   
	  if (count >= write_index - (*f_pos)) {
      count = write_index - (*f_pos);	  
	  }
 	  	  
	  printk(KERN_INFO "my_chardevice: copying %lu bytes from kernel to user\n", count);
    status = copy_to_user(buf, ptr_BUFF_B + (*f_pos), count ); 
	
  	if (status != 0) {  
    	printk(KERN_ERR "Could not copy to user in %s, %i\n",__FUNCTION__,__LINE__);
    	return -1;
    }
	
	  else {
      *f_pos += count;
      printk(KERN_INFO "\nmy_chardevice: DEBUG RD write_index = %lld count = %lu offset = %lld\n\n", write_index, count, (*f_pos));
	    return count;
    }
  }
  /* Nothing to read*/
  else
    return 0; 
}

// Write function //
ssize_t dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {

  int status, overflow_flag = 0;

  ssize_t retvalue=-ENOMEM;
	
  //TODO: Check input parameters?

  /* Validation of user pointers */
	if (filp == NULL || buf == NULL || f_pos == NULL) {
    printk(KERN_ERR "Invalid pointers in %s, %i\n",__FUNCTION__,__LINE__);
    return -1;
	}
  
  //TODO: check condition of count > BUF_SIZE superposition with overflow
  
  /* Validation of count to write */
	if (count < 0 || count > CHAR_BUFFER_SIZE) {
    printk(KERN_ERR "Invalid count of chars to write received in %s, %i\nTruncated to buffer size",__FUNCTION__,__LINE__);
    count = CHAR_BUFFER_SIZE;
	}

  /* Validation of offset to write */
  if ( (* f_pos) < 0 || (* f_pos) > CHAR_BUFFER_SIZE ) {
    printk(KERN_ERR "Invalid offset of chars writen received in %s, %i\n",__FUNCTION__,__LINE__);    
    return -1;
  }

	printk(KERN_INFO "my_chardevice: Writting to BUFF_A: %s", buf);
	//printk(KERN_INFO "my_chardevice: Count parameter from user space %lld.\n", count);
  //printk(KERN_INFO "my_chardevice: Offset in buffer %lld.\n",(long unsigned int)*f_pos);
	  
  // overflow condition
  if ( (*f_pos) + count > CHAR_BUFFER_SIZE) {
  	count = CHAR_BUFFER_SIZE - (*f_pos);
  	overflow_flag = 1; 
  }

  if ( (*f_pos) >= CHAR_BUFFER_SIZE ) {
    printk(KERN_INFO "Buffer is already complete");
    return 0;
  }
  
  //TODO: Revisar si quiero dejar el '\0' al final de cada mensaje o no
  spin_lock(&lock_buffer);
  status = strncpy_from_user( ptr_BUFF_A + (*f_pos), buf, count );
  spin_unlock(&lock_buffer);

  *f_pos += count;
  write_index = *f_pos;
  
  if (status < 0) {
    printk(KERN_ERR "Could not copy from user in %s, %i\n",__FUNCTION__,__LINE__);
    retvalue = EFAULT;
  }
  
  /*
  else if (overflow_flag) { 
    printk(KERN_INFO "my_chardevice: DEBUG WR OVERFLOW count = %lu offset = %lld\n",count,(*f_pos));
    retvalue = CHAR_BUFFER_SIZE + 1;
  }
  */
  else {
    printk(KERN_INFO "my_chardevice: DEBUG WR count = %lu offset = %lld\n",count,(*f_pos));
    retvalue = count;    
  }
  	
  return retvalue;
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
