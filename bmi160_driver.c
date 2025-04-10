//--------------FS --------------//
struct file {
    int flags;
    int pos;
    struct inode *i_node;
    char *priv;
};

struct file_ops_s {
    int (*open)(struct file* filep); //filep = i_node->i_priv +flags + pos
    int (*close)(struct file* filep);
    int (*read)(struct file* filep, char *buffer, int buflen);
    int (*write)(struct file* filep, char *buffer, int buflen);
};

union i_ops_u
{
    struct file_ops_s *ops;
};

struct inode {
    struct inode *parent;
    struct inode *child;
    struct inode *sibling;
    int crefs;
    int flags;

    union i_ops_u *u;
    
    char *i_priv; // dev/xxx/yxz
};

int register_driver(char *devName, struct file_operations *f_ops, int mode, void *priv) {
    struct inode *i = malloc(sizeof(inode));
    if(i==NULL)
        return 1;
    i->i_name = devName; //dev/i2c1
    i->u = f_ops;        //&g_i2c_drvr_ops
    i->flags = mode;     //0666 for char devices
    i->i_priv = priv;    //i2c_master_s
}

//-------------- I2C --------------//
struct i2c_drvr_s
{
    struct i2c_master_s *i2c;
    int cref;
    mutex_t lock;
    bool unlinked;
};

struct file_ops_s g_i2c_drvr_ops = {
    i2c_drvr_open,
    i2c_drvr_close,
    i2c_drvr_read,
    i2c_drvr_write,
    i2c_drvr_ioctl
};

//-------------- I2C-master --------------//
struct i2c_master_s {
    struct i2c_ops_s *ops;
};

struct i2c_ops_s {
    (*transfer) (struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
    (*setup) (struct i2c_master_s *dev, struct config_s *config);
    (*reset) (struct i2c_master_s *dev);
    (*shutdown) (struct i2c_master_s *dev);
};

int i2c_register(struct i2c_master_s *i2c, int bus){
    const char *devName = "/dev/i2c" & "bus"; //generar el devPath
    //register_driver(char *devName, struct file_operations *f_ops, int mode, char *priv)
    register_driver(devName, &g_i2c_drvr_ops, 0666, i2c);
    //open, close, read, write => i2c_drvr_open, i2c_drvr_close, i2c_drvr_read, i2c_drvr_write
    //transfer, setup => i2c_transfer, i2c_setup
}

//-------------- BMI160 --------------//
struct bmi160_dev_s {
    struct i2c_master_s *i2c;
    int address;
    int frq;
};

struct bmi160_fops_s {
    int (*bmi160_open)(struct file* filep);
    int (*bmi160_close)(struct file* filep);
    int (*bmi160_read)(struct file* filep, char *buffer, int buflen);
    int (*bmi160_write)(struct file* filep, char *buffer, int buflen);
};

bmi160_register(struct bmi160_dev_s *dev, struct i2c_master_s *i2c){
    //driver_register(char *devName, struct file_operations *f_ops, int mode, char *priv)

}

int bmi160_open(char* devPath, int flags){
    //create a connection we can use to read/write/...

    //set-up sensor

}

int bmi160_read(int fd, char* buffer, int lenght){
    //
}

int bmi160_write(int fd, char* buffer, int lenght){
    //
}



