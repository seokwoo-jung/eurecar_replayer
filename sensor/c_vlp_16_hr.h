#ifndef C_VLP_16_HR_H
#define C_VLP_16_HR_H

#define VLP_16_HR_DATA_SIZE 1206
#define VLP_16_HR_BOLCKS_NUM 12
#define VLP_16_HR_LASERS_NUM 32

#define VLP_16_HR_VER_START  10.67
#define VLP_16_HR_VER_END   -30.67
#define VLP_16_HR_VER_RESOL   1.33

#define VLP_16_HR_HOR_START  0
#define VLP_16_HR_HOR_END   360
#define VLP_16_HR_HOR_RESOL 0.16

#define VLP_16_HR_TOTAL_PACKET_NUMBER 185 // Averagely PACKET_NUMBER is 180 ~ 181. For safty using 185

#pragma pack(push,1)

#define PI 3.14159265358979
#define D2R (PI/180.)
#define R2D (180./PI)

#define BUFF_SIZE 1250

typedef unsigned long DWORD;
#ifndef BYTE
#define BYTE unsigned char
#endif
typedef unsigned short WORD;
typedef unsigned int UINT;
typedef int INT;


const double vlp_16_hr_firing_vertical_angle[16] = {
    -10, 0.667, -8.667, 2, -7.333, 3.333, -6, 4.667, -4.667, 6.000, -3.333, 7.333, -2, 8.667, -0.667, 10
};


typedef struct _VLP_16_HR_DIST_INTEN_DATA{
    unsigned short distance;
    unsigned char intensity;

}VLP_16_HR_Laser_Data;

typedef struct _VLP_16_HR_SINGLE_FIRING_DATA{

    unsigned short block_id;
    unsigned short rotation;

    VLP_16_HR_Laser_Data laser_data[32];

}VLP_16_HR_FIRING_DATA;

typedef struct _VLP_16_HR_ONE_PACKET{

    VLP_16_HR_FIRING_DATA firing_data[12];

    int gps_time_stamp;
    unsigned char status_type;
    unsigned char status_value;

}VLP_16_HR_DATA;

#pragma pack(pop)

#endif // C_VLP_16_HR_H
