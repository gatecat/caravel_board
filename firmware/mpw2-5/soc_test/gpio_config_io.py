# number of IO in the configuration stream for each chain
NUM_IO = 19

# defines these values for IO configurations
C_MGMT_OUT = 0
C_MGMT_IN = 1
C_USER_BIDIR = 2
C_DISABLE = 3
C_ALL_ONES = 4
C_USER_BIDIR_WPU = 5
C_USER_BIDIR_WPD = 6
C_USER_IN_NOPULL = 7
C_USER_OUT = 8

config_h = [
    C_USER_OUT,  #37
    C_DISABLE,  #36
    C_DISABLE,  #35
    C_DISABLE,  #34
    C_USER_OUT,  #33
    C_USER_OUT,  #32
    C_USER_OUT,  #31
    C_USER_OUT,  #30
    C_USER_OUT,  #29
    C_USER_BIDIR_WPU,  #28
    C_USER_BIDIR_WPU,  #27
    C_USER_BIDIR_WPD,  #26
    C_USER_BIDIR_WPD,  #25
    C_USER_BIDIR_WPD,  #24
    C_USER_BIDIR_WPD,  #23
    C_USER_BIDIR_WPD,  #22
    C_USER_BIDIR_WPD,  #21
    C_USER_BIDIR_WPD,  #20
    C_USER_BIDIR_WPD,  #19
]

del config_h[NUM_IO:]

config_l = [
    C_USER_IN_NOPULL,   #0
    C_USER_IN_NOPULL,   #1
    C_USER_OUT,   #2
    C_USER_OUT,   #3
    C_USER_BIDIR_WPD,   #4
    C_USER_BIDIR_WPD,   #5
    C_USER_BIDIR_WPD,   #6
    C_USER_BIDIR_WPD,   #7
    C_USER_BIDIR_WPD,   #8
    C_USER_BIDIR_WPD,   #9
    C_USER_BIDIR_WPD,   #10
    C_USER_BIDIR_WPD,   #11
    C_USER_BIDIR_WPD,   #12
    C_USER_BIDIR_WPD,   #13
    C_USER_BIDIR_WPD,   #14
    C_USER_BIDIR_WPD,   #15
    C_USER_BIDIR_WPD,   #16
    C_USER_BIDIR_WPD,   #17
    C_USER_BIDIR_WPD,   #18
]

del config_l[NUM_IO:]