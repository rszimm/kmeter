#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/kthread.h>
#include <linux/delay.h>      // sleep functions
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
       #include <linux/sched/signal.h>
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Zimmerman");
MODULE_DESCRIPTION("a Sensus meter reading Kernel Module");
MODULE_VERSION("0.1");

static int dataPin[4]={-1,-1,-1,-1};
static int dataPin_argc = 0;
module_param_array(dataPin, int, &dataPin_argc, S_IRUGO);
MODULE_PARM_DESC(dataPin, "The Data pin for each meter");

static int clockPin[4]={-1,-1,-1,-1};
static int clockPin_argc = 0;
module_param_array(clockPin, int, &clockPin_argc, S_IRUGO);
MODULE_PARM_DESC(clockPin, "The Clock pin for each meter (you can share!)");

static int debug=0;
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "set nonzero to debug output data");

static int poll_interval=30;
module_param(poll_interval, int, S_IRUGO);
MODULE_PARM_DESC(poll_interval, "Meter Polling Interval in Seconds");

// Size of the received data array
#define DATA_ARRAY_SIZE 100
// Clock period is twice the TICK_TIME (in uS)
#define TICK_TIME 500
// Time we wait for the input to settle (in uS)
#define SETTLE_TIME 70

// main structure that defines a particular water meter
static struct meterspec 
{
    // readable data values
    s32 value;
    s32 number;
    s32 k_number;
    time_t last_read_time;
    // config values
    int dataPin;
    int clockPin;

    // internals
    struct kobject* kobj;           // location for storing sysfs data
    char data[DATA_ARRAY_SIZE];     // Received data buffer
    int data_index;                 // index into received data buffer
    int bitno;                      // bit position we're currently reading
    bool parity_check;              // parity check bit
    bool isDone;                    // true if we're all done reading this meter
    enum STATE {WAIT_FOR_START, READ_BITS, WAIT_FOR_PARITY, WAIT_FOR_STOP} state;
} mspec[4];

DEFINE_MUTEX(mspec_mutex);  // for managing access to the shared parts of mspec

static ssize_t value_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%d", mspec[index].value);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%d", mspec[index].number);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t k_number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%d", mspec[index].k_number);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t dataPin_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%d", mspec[index].dataPin);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t clockPin_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%d", mspec[index].clockPin);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t lastTime_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
    int index;
    ssize_t retval = 0;
    sscanf(kobj->name, "%d", &index);
    if ((index < 0) || (index >= dataPin_argc))
        return 0;
    mutex_lock(&mspec_mutex);
    retval = scnprintf(buf, PAGE_SIZE, "%lu", mspec[index].last_read_time);
    mutex_unlock(&mspec_mutex);
    return retval;
}

static ssize_t poll_interval_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d", poll_interval);
}

static ssize_t poll_interval_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &(poll_interval));
    return count;
}

static struct kobj_attribute value_attr  = __ATTR_RO(value);
static struct kobj_attribute number_attr  = __ATTR_RO(number);
static struct kobj_attribute k_number_attr = __ATTR_RO(k_number);
static struct kobj_attribute time_attr  = __ATTR_RO(lastTime);
static struct kobj_attribute clockPin_attr  = __ATTR_RO(clockPin);
static struct kobj_attribute dataPin_attr  = __ATTR_RO(dataPin);
static struct kobj_attribute poll_interval_attr = __ATTR(poll_interval, 0664, poll_interval_show, poll_interval_store);


static struct attribute *sysfs_attrs[] = {
    &value_attr.attr,
    &number_attr.attr,
    &time_attr.attr,
    &k_number_attr.attr,
    &clockPin_attr.attr, 
    &dataPin_attr.attr,
    NULL,
};

static struct attribute_group sysfs_group = {
    .attrs = sysfs_attrs, 
};

static struct kobject *sysfs_kobj;

void reset_meter_buffer(int mtr_num)
{
    mspec[mtr_num].data_index = 0;
    mspec[mtr_num].bitno = 0;
    mspec[mtr_num].parity_check = false;
    mspec[mtr_num].isDone = false;
    mspec[mtr_num].state = WAIT_FOR_START;
}

void reset_all_meters(void)
{
    int i;
    for (i=0; i<dataPin_argc; i++)
        reset_meter_buffer(i);
}

// Initialize a blank meter
void init_meter(int mtr_num, int dataPin, int clockPin)
{
    mspec[mtr_num].value = -1;
    mspec[mtr_num].number = -1;
    mspec[mtr_num].k_number = -1;
    mspec[mtr_num].last_read_time = 0;

    mspec[mtr_num].clockPin = clockPin;
    mspec[mtr_num].dataPin = dataPin;

    mspec[mtr_num].kobj = 0;
    reset_meter_buffer(mtr_num);
}

// release an allocated meter
void free_meter(int mtr_num)
{
    if ((mtr_num < 0) || (mtr_num >= 4))
        return;
    if (mspec[mtr_num].kobj)
        kobject_put(mspec[mtr_num].kobj);
    gpio_unexport(mspec[mtr_num].dataPin);
    gpio_free(mspec[mtr_num].dataPin);
    gpio_unexport(mspec[mtr_num].clockPin);
    gpio_free(mspec[mtr_num].clockPin);
}

static struct task_struct *task;             // The pointer to the thread task

static int mainTask(void* data);

static int __init kMeter_init(void){
    enum dat_pin_val {NOT_ASSIGNED = 0, ASSIGNED_DATA, ASSIGNED_CLOCK} pinval[100] ={0};
    int result = 0;
    int i;
    char counterNum[4];

    printk(KERN_INFO "kMeter: Initializing the kMeter LKM\n");

    // create the kobject sysfs entry at /sys/ebb 
    sysfs_kobj = kobject_create_and_add("kMeter", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
    if(!sysfs_kobj)
    {
        printk(KERN_ALERT "kMeter: failed to create kobject mapping\n");
        return -ENOMEM;
    }
    result = sysfs_create_file(sysfs_kobj, &poll_interval_attr.attr);
    if (result)
    {
        printk(KERN_ALERT "kMeter: failed to add kobject attribute\n");
        return result;
    }

    for (i=0; i<dataPin_argc; i++)
    {
        if (debug)
            printk(KERN_INFO "kMeter: Meter %d Data %d Clock %d\n", i, dataPin[i], clockPin[i]);
        if ((!gpio_is_valid(dataPin[i])) || (dataPin[i] < 0) || (dataPin[i] >= 100) 
            || (!gpio_is_valid(clockPin[i])) || (clockPin[i] < 0) || (clockPin[i] >= 100))
        {
            printk(KERN_ALERT "kMeter: Illegal Pin Config Meter %d\n", i);
            for(i--; i>=0;i--)
                free_meter(i);
            kobject_put(sysfs_kobj);  // remove sysfs entry
            return -EINVAL;
        }
        if (pinval[dataPin[i]] != NOT_ASSIGNED)
        {
            printk(KERN_ALERT "kMeter: Duplicate Data Pin %d in Meter %d\n", dataPin[i], i);
            for(i--; i>=0;i--)
                free_meter(i);
            kobject_put(sysfs_kobj);  // remove sysfs entry
            return -EINVAL;
        }
        pinval[dataPin[i]] = ASSIGNED_DATA;
        if (pinval[clockPin[i]] == ASSIGNED_DATA)
        {
            printk(KERN_ALERT "kMeter: Clock Pin Assigned to Data Pin %d in Meter %d\n", clockPin[i], i);
            for(i--; i>=0;i--)
                free_meter(i);
            kobject_put(sysfs_kobj);  // remove sysfs entry
            return -EINVAL;
        }
        else
            pinval[clockPin[i]] = ASSIGNED_CLOCK;

        init_meter(i, dataPin[i], clockPin[i]);

        sprintf(counterNum, "%d", i);
        mspec[i].kobj = kobject_create_and_add(counterNum, sysfs_kobj);
        if (!mspec[i].kobj)
        {
            printk(KERN_ALERT "kMeter: failed to create kobject mapping\n");
            for(i--; i>=0;i--)
                free_meter(i);
            kobject_put(sysfs_kobj);                          // clean up -- remove the kobject sysfs entry
            return -EINVAL;
        }
        // add the attributes to the counter
        result = sysfs_create_group(mspec[i].kobj, &sysfs_group);
        if(result) {
            printk(KERN_ALERT "kMeter: failed to create sysfs group\n");
            for(; i>=0;i--)
                free_meter(i);
            kobject_put(sysfs_kobj);                          // clean up -- remove the kobject sysfs entry
            return result;
        }
        gpio_request(mspec[i].dataPin, "dataPin");       // Set up the dataPin
        gpio_direction_input(mspec[i].dataPin);        // Set the dataPin to be an input
        gpio_export(mspec[i].dataPin, true);          // Causes dataPin to appear in /sys/class/gpio
        gpio_request(mspec[i].clockPin, "clockPin");       // Set up the clockPin
        gpio_direction_output(mspec[i].clockPin, 0);        // Set the clockPin to be an output
        gpio_export(mspec[i].clockPin, true);          // Causes clockPin to appear in /sys/class/gpio
    }
    task = kthread_run(mainTask, NULL, "kMeter_thread");
    return result;
}

/** @brief The LKM cleanup function
*  Similar to the initialization function, it is static. The __exit macro notifies that if this
*  code is used for a built-in driver (not a LKM) that this function is not required.
*/
static void __exit kMeter_exit(void){
    int i;

    printk(KERN_INFO "kMeter: Requesting thread stop\n");
    if (task)
        kthread_stop(task);

    for (i=0; i<dataPin_argc; i++)
        free_meter(i);
    kobject_put(sysfs_kobj);  // remove sysfs entry
    printk(KERN_INFO "kMeter: Leaving Kernel Module\n");
}


void parse_data(struct meterspec* p)
{
    // data comes in as V;RBxxxxxxx;IByyyyy;Kmmmmm
    //  where xxxx is the meter read value (arbitrary digits, but not more than 12)
    //  yyyy is the meter id (arbitrary digits)
    //  mmmm is another meter id (arbitrary digits)
    //  Note that the IB and K parts are optional

    struct timespec ts;
    enum STATE {PARSE_V, PARSE_SEMI, PARSE_PRE, PARSE_NUM} state = PARSE_V;
    char* data_ptr = p->data;
    // temp storage for variables until we've parsed the whole string.
    s32 value = -1;
    s32 number = -1;
    s32 k_number = -1;
    s32* num_ptr = &(k_number);
    while (*data_ptr != 0)
    {
//        if (debug)
//            printk(KERN_INFO "s%d, c%02d\n", state, *data_ptr);
        switch (state)
        {
        case PARSE_V:
            if (*data_ptr != 'V')
                goto err_exit;
            state = PARSE_SEMI;
            break;
        case PARSE_SEMI:
            if (*data_ptr != ';')
                goto err_exit;
            state = PARSE_PRE;
            break;
        case PARSE_PRE:
            if ((*data_ptr == 'R') && (*(data_ptr + 1) == 'B'))
            {
                num_ptr = &value;
                data_ptr ++;
            }
            else if ((*data_ptr == 'I') && (*(data_ptr + 1) == 'B'))
            {
                num_ptr = &number;
                data_ptr++;
            }
            else if ((*data_ptr == 'K'))
                num_ptr = &k_number;
            *num_ptr = 0;
            state = PARSE_NUM;
            break;
        case PARSE_NUM:
        {
            if (((*data_ptr) >= 48) && ((*data_ptr) <= 57))
            {
                *num_ptr = (*num_ptr) * 10 + (*data_ptr) - 48;
                break;
            }
            data_ptr--;
            state = PARSE_SEMI;
        }
        }
        data_ptr++;
    }
    mutex_lock(&mspec_mutex);
    p->value = value;
    p->number = number;
    p->k_number = k_number;
    getnstimeofday(&ts);
    p->last_read_time = ts.tv_sec;
    mutex_unlock(&mspec_mutex);
    if (debug)
        printk(KERN_INFO "kMeter:%d,%d,%d\n", p->value, p->number, p->k_number);
    return;
err_exit:
    if (debug)
        printk(KERN_INFO "Parse Error\n");
    return;
}

// Sets all the clock pins to the value passed in
static void setClockPin(bool value)
{
    int i;
    for (i=0; i<dataPin_argc; i++)
        if ((mspec[i].clockPin >=0) && (!(mspec[i].isDone)))
            gpio_set_value(mspec[i].clockPin, value);
}

static void update_state(int mtr_num)
{
    bool input = gpio_get_value(mspec[mtr_num].dataPin);
    struct meterspec* p = &(mspec[mtr_num]);
    //if (debug)
    //    printk(KERN_INFO "kMeter: g %d s %d i %d\n", input, p->state, p->data_index);

    switch (p->state)
    {
    case WAIT_FOR_START: // LOST_SYNC
        if (!input)  // wait unto we get a 0
        {
            p->data[p->data_index] = 0;
            p->bitno = 0;
            p->parity_check = false;
            p->state = READ_BITS;
        }
        break;
    case READ_BITS:
        if (input)
        {
            p->data[p->data_index] |= 1 << p->bitno;
            p->parity_check = !(p->parity_check);
        }
        if ((++(p->bitno)) == 7)
            p->state = WAIT_FOR_PARITY;
        break;
    case WAIT_FOR_PARITY:
        if ((!input) != (!(p->parity_check)))
        {
            if (debug)
                printk(KERN_INFO "kMeter: Meter %d Parity Error\n", mtr_num);
            reset_meter_buffer(mtr_num);
            return;
        }
        p->state = WAIT_FOR_STOP;
        break;
    case WAIT_FOR_STOP:
        p->state = WAIT_FOR_START;
        if (!input)
        {
            if (debug)
                printk(KERN_INFO "kMeter: Meter %d Lost Sync\n", mtr_num);
            reset_meter_buffer(mtr_num);
            return;
        }
        if (p->data[p->data_index] == 13)
        {
            p->data[p->data_index] = 0;
            if (debug)
                printk(KERN_INFO "kMeter: Meter %d, Data:%s\n", mtr_num, p->data);
            parse_data(p);
            p->isDone = true;
            break;
        }
        if ((++(p->data_index)) >= DATA_ARRAY_SIZE)
        {
            if (debug)
                printk(KERN_INFO "kMeter: Meter %d Buffer Overflow\n", mtr_num);
            reset_meter_buffer(0);
            return;
        }
        break;
    }
}

// returns true if all the meters have been read and we can safely go to sleep until the next polling interval
bool mainLoop(void)
{
    static bool tick_tock = false;
    int i;
    bool allMetersDone = true;

    tick_tock = !tick_tock;
    if (!tick_tock)
    {
        setClockPin(true);
        // wait for the data pins to settle
        usleep_range(SETTLE_TIME, SETTLE_TIME+10);

        // Update all the meters
        for (i=0; i<dataPin_argc; i++)
        {
            if (!mspec[i].isDone)
                update_state(i);
            if (!mspec[i].isDone)
                allMetersDone = false;
        }

        if (allMetersDone)
            return true;
    }
    else
        setClockPin(false);
    return false;
}

static int mainTask(void* data)
{
    unsigned long jiffies_timeout;
    bool allDone;
    allow_signal(SIGKILL);
    //set_current_state(TASK_INTERRUPTIBLE);
    while (!kthread_should_stop())
    {
        //set_current_state(TASK_RUNNING);
        allDone = mainLoop();
        jiffies_timeout = msecs_to_jiffies(poll_interval * 1000) + 1;
        set_current_state(TASK_INTERRUPTIBLE);
        if (allDone)
        {
            reset_all_meters();
            setClockPin(false);  // turn off clock pins so meters all reset to their resting state.
            schedule_timeout(jiffies_timeout);
        }
        else
            usleep_range(TICK_TIME, TICK_TIME + 100);
    }
    return 0;
}

// Tie in the initialization and exit functions so the kernel can install/remove us
module_init(kMeter_init);
module_exit(kMeter_exit);
