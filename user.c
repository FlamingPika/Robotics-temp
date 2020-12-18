#include "user.h"
#include "emulator.h"
#include "qdbmp.h"
#include <time.h>
#include <math.h>

#define pi 3.141592654

extern uint64_t iter;  

void setup(void)
{
    /*
    //emwrite(2, 0, 430, 515, pi/2); //White Track
    //emwrite(2, 0, 50, 50, 3*pi/2); //Origin
    emwrite(2, 0, 50, 480, 4*pi/2); //Rack
    //emwrite(2, 0, 435, 260, 3*pi/2); //Ball Basket
    emwrite(2, camera, 0, 0, pi);
    emwrite(13, camera, 0, 1);
    emwrite(13, camera, 1, 2.95*pi/8);
    emwrite(2, line1, 25, 40, 0);
    emwrite(2, line2, -25, 40, 0);
    emwrite(2, line3, 0, 40, 0);
    emwrite(2, magnetic1, 7, 40, 0);
    emwrite(2, magnetic2, -7, 40, 0);
    emwrite(2, magnetic3, 20, 40, 0);
    emwrite(2, magnetic4, -20, 40, 0);
    emwrite(2, ir1, 5, 0, 12.2*pi/16);
    emwrite(2, ir2, -5, 0, pi);
    emwrite(2, grab, 0, 0, pi);
    */
    emwrite(2, 0, 435, 260, 3*pi/2); //Ball Basket
    //emwrite(2, 0, 50, 480, 0);
    emwrite(2, camera, 0, 40, pi/2);
    emwrite(13, camera, 0, 1);
    emwrite(13, camera, 1, 2.95*pi/8);
    emwrite(2, line1, 15, 40, 0);
    emwrite(2, line2, -15, 40, 0);
    emwrite(2, line3, 0, -40, 0);
    emwrite(2, magnetic1, 7, 40, 0);
    emwrite(2, magnetic2, -7, 40, 0);
    emwrite(2, magnetic3, 20, 40, 0);
    emwrite(2, magnetic4, -20, 40, 0);
    emwrite(2, ir1, 20, 40, pi*8/9 );
    emwrite(2, ir2, -20, 40, pi*7/6);
    emwrite(2, grab, 0, 40, pi);
    emwrite(2, throw, 0, 40, pi);
    emwrite(1, camera);
    emwrite(1, line1);
    emwrite(1, line2);
    emwrite(1, line3);
    emwrite(1, magnetic1);
    emwrite(1, magnetic2);
    emwrite(1, magnetic3);
    emwrite(1, magnetic4);
    emwrite(1, ir1);
    emwrite(1, ir2);
    emwrite(1, motor1);
    emwrite(1, motor2);
    emwrite(1, grab);
    emwrite(1, throw);
    
    //  N.B. You can only initialise and configure components within this function.
    //       (Of course, you can still call another function which does the init.)
}

bool is_data_requested = false;
bool is_data_requested_line = false;
bool is_data_requested_magnetic = false;
bool is_data_requested_ir = false;

bool turned = false;
float last_error1 = 0;
float sum1 = 0;
float last_error2 = 0;
float sum2 = 0;
float volt_a = -24.0;
float volt_b = -24.0;
float kp1 = 0;
float kd1 = 1;
float kp2 = 0;
float kd2 = 0;


void PID_FUNCTION(int *m1, int *m2, int *m3, int *m4)
{
    /*
    float kp1 = 1.3;
    float kd1 = 0;
    float kp2 = 100;
    float kd2 = 0;
    */
    float error1;
    float p1;
    float d1;
    float derivative1;
    float correction1;
    float error2;
    float p2;
    float d2;
    float derivative2;
    float correction2;

    /*
    if (*m4 > 140){
      kp1 = 0;
      kp2 = 100;
      kd1 = 0;
    }
    else{
      kp1 = 1.3;
      kp2 = 0;
      kd2 = 0;
      error2 = 0;
      correction2 = 0;
    }
    */

    emdebug("Last Error1 = %f\n", last_error1);
    emdebug("kp1: %f\n", kp1);
    
    error1 = *m1 - *m2;
    emdebug("Error1: %f\n", error1);

    p1 = error1*kp1;
    emdebug("P1: %f\n", p1);

    derivative1 = error1 - last_error1;
    d1 = derivative1*kd1;
    emdebug("D1: %f\n", d1);

    correction1 = p1 + d1;
    correction1 = correction1*24/200;
    emdebug("correction1 = %f\n", correction1);

    last_error1 = error1;
    sum1 = sum1 + error1;
    emdebug("Last Error2 = %f\n", last_error2);
    emdebug("kp2: %f\n", kp2);

    error2 = *m3 - *m4;
    emdebug("Error2: %f\n", error2);

    p2 = error2*kp2;
    emdebug("P2: %f\n", p2);

    derivative2 = error2 - last_error2;
    d2 = derivative2*kd2;
    emdebug("D2: %f\n", d2);

    correction2 = p2 + d2;
    correction2 = correction2*24/200;
    emdebug("correction2 = %f\n", correction2);

    last_error2 = error2;
    sum2 = sum2 + error2;
    volt_a = -24 + correction1 + correction2;
    volt_b = -24 - correction1 - correction2;
    
    return;
}

bool MOVE_TO_BASKET_FUNC()
{
    emdebug("Volt1 = %f\n", volt_a);
    emdebug("Volt2 = %f\n", volt_b);
    int m1, m2, m3, m4;
    bool obstacle1 = false, obstacle2 = false;
    if(is_data_requested_magnetic){
        int id1 = magnetic1;
        if(emread_magnetic_sensor(&id1, &m1)){
            emdebug("M1 = %d\n", m1);
        }
        int id2 = magnetic2;
        if(emread_magnetic_sensor(&id2, &m2)){
            emdebug("M2 = %d\n", m2);
        }
        int id3 = magnetic3;
        if(emread_magnetic_sensor(&id3, &m3)){
            emdebug("M3 = %d\n", m3);
        }
        int id4 = magnetic4;
        if(emread_magnetic_sensor(&id4, &m4)){
            emdebug("M4 = %d\n", m4);
        }
    }

    if(is_data_requested_ir){
        int idr1 = ir1;
        emdebug("HIHIHI\n");
        if(emread_ir_sensor(&idr1, &obstacle1)){
            emdebug("IR1: %s\n",obstacle1 ? "true" : "false");
        }
        int idr2 = ir2;
        if(emread_ir_sensor(&idr2, &obstacle2)){
            emdebug("IR2: %s\n",obstacle2 ? "true" : "false");
        }  
    }

    if( obstacle1 == true && obstacle2 == true){
        emdebug("VOLT A HAS BEEN REDUCED TO 0\n");
        emdebug("VOLT B HAS BEEN REDUCED TO 0\n");
        volt_a = 0;
        volt_b = 0;
        is_data_requested_magnetic = false;
        is_data_requested_ir = false;
        return true;
    }

    else{
        emwrite(4, magnetic1);
        emwrite(4, magnetic2);
        emwrite(4, magnetic3);  
        emwrite(4, magnetic4);

        emwrite(4, ir1);
        emwrite(4, ir2);

        is_data_requested_magnetic = true;
        is_data_requested_ir = true;

        emdebug("%d %d %d %d\n", m1,m2,m3,m4);
        //PID_FUNCTION(&m1,&m2,&m3,&m4);

        return false;
    }
    
    
    /*
    if(m1 < 30 && m2 < 30 && m3 < 30 && m4 < 30){
        emdebug("VOLT A HAS BEEN REDUCED TO 0\n");
        emdebug("VOLT B HAS BEEN REDUCED TO 0\n");
        volt_a = 0;
        volt_b = 0;
        is_data_requested_magnetic = false;
        return true;
    }

    else{
        return false;
    }
    */
    
    
}

bool GRAB_EMPTY_BASKET_FUNC()
{
    int grab_state = 0;
    emwrite(10, 3);
    emdebug("GRABBED\n");
    grab_state += 1;

    if (grab_state >= 1){
        return true;
    }
    else{
        return false;
    }
}

int finish_sb = 0;
bool basket2 = false;
bool backward1 = false;
bool backward2 = false;
bool backward3 = false;
bool turn1 = false;
bool turn2 = false;
bool forward1 = false;
bool backward4 = false;
bool backward5 = false;
bool turn3 = false;

bool MOVE_TO_SECOND_BASKET_FUNC()
{
    int m1, m2, m3, m4;
    bool l1, l2, l3;
    bool obstacle1, obstacle2;

    if(is_data_requested_magnetic){
        int id1 = magnetic1;
        if(emread_magnetic_sensor(&id1, &m1)){
            emdebug("M1 = %d\n", m1);
        }
        int id2 = magnetic2;
        if(emread_magnetic_sensor(&id2, &m2)){
            emdebug("M2 = %d\n", m2);
        }
        int id3 = magnetic3;
        if(emread_magnetic_sensor(&id3, &m3)){
            emdebug("M3 = %d\n", m3);
        }
        int id4 = magnetic4;
        if(emread_magnetic_sensor(&id4, &m4)){
            emdebug("M4 = %d\n", m4);
        }
    }

    if(is_data_requested_ir){
        int id1 = ir1;
        if(emread_ir_sensor(&id1, &obstacle1)){
            emdebug("IR1: %s\n",obstacle1 ? "true" : "false");
        }
        int id2 = ir2;
        if(emread_ir_sensor(&id2, &obstacle2)){
            emdebug("IR2: %s\n",obstacle2 ? "true" : "false");
        }
    }
    if(is_data_requested_line){
        int id1 = line1;
        if(emread_line_sensor(&id1, &l1)){
            emdebug("Line1: %s\n",l1 ? "true" : "false");
        }
        int id2 = line2;
        if(emread_line_sensor(&id2, &l2)){
            emdebug("Line2: %s\n",l2 ? "true" : "false");
        }
        int id3 = line3;
        if(emread_line_sensor(&id3, &l3)){
            emdebug("Line3: %s\n",l3 ? "true" : "false");
        }
    }
    
    if (finish_sb == 1){
        is_data_requested_line = false;
        is_data_requested_magnetic = false;    
        is_data_requested_ir = false;  
        return true;
    }
    else{
        emwrite(4, magnetic1);
        emwrite(4, magnetic2);
        emwrite(4, magnetic3);
        emwrite(4, magnetic4);
        
        emwrite(4, ir1);
        emwrite(4, ir2);
        
        emwrite(4, line1);
        emwrite(4, line2);
        emwrite(4, line3);

        is_data_requested_magnetic = true;
        is_data_requested_ir = true;
        is_data_requested_line = true;

        if(basket2 == false){
            if(backward1 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward1 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(backward2 == false){
                if(l3 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward2 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(backward3 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward3 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(turn1 == false){
                if(l2 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    turn1 = true;
                }
                else{
                    volt_a = 0;
                    volt_b = -10;
                }
            }
            else if(turn2 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    turn2 = true;
                }
                else{
                    volt_a = 0;
                    volt_b = -10;
                }
            }
            else if(forward1 == false){
                if(obstacle1 == 1 && obstacle2 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    emwrite(10, 3);
                    forward1 = true;
                }
                else{
                    volt_a = -24;
                    volt_b = -24;
                }
            }
            else if(backward4 == false){
                if(obstacle1 == 0){
                    volt_a = 0;
                    volt_b = 0;
                    backward4 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(backward5 == false){
                if(l3 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward5 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(turn3 == false){
                if(m1 > 80){
                    
                    volt_a = -24;
                    volt_b = -24;
                    turn3 = true;

                }
                else{
                    volt_a = -12;
                    volt_b = 5;
                }
            }
            else if(turn3 == true){
                if(m1 < 10 && m2 < 10 && m3 < 10 && m4 < 10){
                    volt_a = 0;
                    volt_b = 0;
                    finish_sb += 1;
                }
            }
        }
        else{
            volt_a = 0;
            volt_b = 0;            
            
        }
        emdebug("Volt1 = %f\n", volt_a);
        emdebug("Volt2 = %f\n", volt_b);
        return false;
    }
}
int pass = 0;
int finish_wt = 0;
bool white_track = false;
bool backward_1 = false;
bool backward_2 = false;
bool backward_3 = false;
bool turn_1 = false;
bool turn_2 = false;
bool turn_3 = false;
bool turn_4 = false;
bool forward_1 = false;
bool forward_2 = false;
bool forward_3 = false;
bool backward_4 = false;
bool backward_5 = false;

bool MOVE_TO_WHITE_TRACK_FUNC()
{
    int m1, m2, m3, m4;
    bool l1, l2, l3;
    bool obstacle1, obstacle2;

    if(is_data_requested_magnetic){
        int id1 = magnetic1;
        if(emread_magnetic_sensor(&id1, &m1)){
            emdebug("M1 = %d\n", m1);
        }
        int id2 = magnetic2;
        if(emread_magnetic_sensor(&id2, &m2)){
            emdebug("M2 = %d\n", m2);
        }
        int id3 = magnetic3;
        if(emread_magnetic_sensor(&id3, &m3)){
            emdebug("M3 = %d\n", m3);
        }
        int id4 = magnetic4;
        if(emread_magnetic_sensor(&id4, &m4)){
            emdebug("M4 = %d\n", m4);
        }
    }

    if(is_data_requested_ir){
        int id1 = ir1;
        if(emread_ir_sensor(&id1, &obstacle1)){
            emdebug("IR1: %s\n",obstacle1 ? "true" : "false");
        }
        int id2 = ir2;
        if(emread_ir_sensor(&id2, &obstacle2)){
            emdebug("IR2: %s\n",obstacle2 ? "true" : "false");
        }
    }
    if(is_data_requested_line){
        int id1 = line1;
        if(emread_line_sensor(&id1, &l1)){
            emdebug("Line1: %s\n",l1 ? "true" : "false");
        }
        int id2 = line2;
        if(emread_line_sensor(&id2, &l2)){
            emdebug("Line2: %s\n",l2 ? "true" : "false");
        }
        int id3 = line3;
        if(emread_line_sensor(&id3, &l3)){
            emdebug("Line3: %s\n",l3 ? "true" : "false");
        }
    }
    
    if (finish_wt == 1){
        is_data_requested_line = false;
        is_data_requested_magnetic = false;    
        is_data_requested_ir = false;  
        return true;
    }
    else{
        emwrite(4, magnetic1);
        emwrite(4, magnetic2);
        emwrite(4, magnetic3);
        emwrite(4, magnetic4);
        
        emwrite(4, ir1);
        emwrite(4, ir2);
        
        emwrite(4, line1);
        emwrite(4, line2);
        emwrite(4, line3);

        is_data_requested_magnetic = true;
        is_data_requested_ir = true;
        is_data_requested_line = true;

        if(white_track == false){
            if(backward_1 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward_1 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(backward_2 == false){
                if(l3 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward_2 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            else if(backward_3 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    backward_3 = true;
                }
                else{
                    volt_a = 10;
                    volt_b = 10;
                }
            }
            
            else if(turn_1 == false){
                if(l2 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    turn_1 = true;
                }
                else{
                    volt_a = 0;
                    volt_b = -16;
                }
            }
            else if(turn_2 == false){
                if(l1 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    turn_2 = true;
                }
                else{
                    volt_a = 0;
                    volt_b = -10;
                }
            }
            else if(forward_1 == false){
                if(obstacle1 == 1 && obstacle2 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    forward_1 = true;
                    
                }
                else{
                    volt_a = -24;
                    volt_b = -24;
                }
            }
            else if(turn_3 == false){
                if(obstacle1 == 0 && obstacle2 == 0){
                    volt_a = -24;
                    volt_b = -24;
                    turn_3 = true;
                    
                    
                }
                else{
                    volt_a = -10;
                    volt_b = 0;
                }
            }
            else if(forward_2 == false){
                if(l2 == 1){
                    volt_a = 0;
                    volt_b = 0;
                    forward_2 = true;
                    return true;
                    
                }
                else{
                    volt_a = -24;
                    volt_b = -24;
                }
            }
            else if(turn_4 == false){
                if (obstacle2 == 0 || obstacle1 ==0){
                    pass = 1;
                }
                if(obstacle2 == 1 && pass == 1){
                    volt_a = 0;
                    volt_b = 0;
                    pass = 0;
                    turn_4 = true;
                    
                }
                else{
                    volt_a = -0;
                    volt_b = -10;
                }
            }
            else if(forward_3 == false){
                emdebug("============== FORWARD 3 =============\n");
                if (obstacle2 == 0){
                    emdebug("============== PASSSSSSSSS =============\n");
                    pass = 1;
                    
                }
                if(obstacle1 == 1 && pass == 1){
                    volt_a = 0;
                    volt_b = 0;
                    forward_3 = true;
                    finish_wt +=1;
                }
                else{
                    volt_a = 0;
                    volt_b = -12;
                }
            }
        }
        else{
            volt_a = 0;
            volt_b = 0;            
            
        }
        emdebug("Volt1 = %f\n", volt_a);
        emdebug("Volt2 = %f\n", volt_b);
        return false;
    }
}

/*COLOR SETUP*/
bool r1 = false;
bool r2 = false;
bool b1 = false;
bool b2 = false;
int x = 1;
UCHAR r, g, b;
UINT tr = 0, tg = 0, tb = 0;
void checkcolor(int tr, int tb, bool *r, bool *b)
{
    if (tr > 1000)

        *r = true;
    else if (tb > 1000)
        *b = true;
}

void sort(UCHAR array[])
{

    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 8 - i; j++)
        {
            if (array[j] > array[j + 1])
            {
                int temp = array[j + 1];
                array[j + 1] = array[j];
                array[j] = temp;
            }
        }
    }
    // for (int i = 0; i < 9; i++)
    // {
    //     emdebug("%d\n", array[i]);
    // }
}

void image()
{
    BMP *bmp;
    UCHAR R[9];
    UCHAR G[9];
    UCHAR B[9];
    uint8_t window[9];

    if (!emread_camera(&bmp))
    {
        emdebug("Failed to read camera data!\n");
        return;
    }
    else
    {

        UINT height = BMP_GetHeight(bmp);
        UINT width = BMP_GetWidth(bmp);
        BMP *bmp1 = BMP_Create(width, height, 32);

        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                BMP_GetPixelRGB(bmp, i - 1, j - 1, &r, &g, &b);
                R[0] = r;
                G[0] = g;
                B[0] = b;
                BMP_GetPixelRGB(bmp, i - 1, j, &r, &g, &b);
                R[1] = r;
                G[1] = g;
                B[1] = b;
                BMP_GetPixelRGB(bmp, i + 1, j - 1, &r, &g, &b);
                R[2] = r;
                G[2] = g;
                B[2] = b;
                BMP_GetPixelRGB(bmp, i - 1, j, &r, &g, &b);
                R[3] = r;
                G[3] = g;
                B[3] = b;
                BMP_GetPixelRGB(bmp, i, j, &r, &g, &b);
                R[4] = r;
                G[4] = g;
                B[4] = b;
                BMP_GetPixelRGB(bmp, i + 1, j, &r, &g, &b);
                R[5] = r;
                G[5] = g;
                B[5] = b;
                BMP_GetPixelRGB(bmp, i - 1, j + 1, &r, &g, &b);
                R[6] = r;
                G[6] = g;
                B[6] = b;
                BMP_GetPixelRGB(bmp, i, j + 1, &r, &g, &b);
                R[7] = r;
                G[7] = g;
                B[7] = b;
                BMP_GetPixelRGB(bmp, i + 1, j + 1, &r, &g, &b);
                R[8] = r;
                G[8] = g;
                B[8] = b;

                sort(R);
                sort(G);
                sort(B);

                BMP_SetPixelRGB(bmp1, i, j, R[4], G[4], B[4]);
            }
        }

        BMP_WriteFile(bmp1, "output.bmp");

        BMP *inputFilter = BMP_ReadFile("output.bmp");
        BMP *outputFilter = BMP_Create(width, height, 8);
        
        for (int i=0; i<256; i++) {
    	    BMP_SetPaletteColor(outputFilter, i, i, i, i);
        }
        
        for (int x=1; x<width-1; x++) {
            for (int y=1; y<height-1; y++) {
                UINT i = 0;
                window[i++] = BMP_GetPixelGray(inputFilter, x-1, y-1);
                window[i++] = BMP_GetPixelGray(inputFilter, x, y-1);
                window[i++] = BMP_GetPixelGray(inputFilter, x+1, y-1);
                window[i++] = BMP_GetPixelGray(inputFilter, x-1, y);
                window[i++] = BMP_GetPixelGray(inputFilter, x, y);
                window[i++] = BMP_GetPixelGray(inputFilter, x+1, y);
                window[i++] = BMP_GetPixelGray(inputFilter, x-1, y+1);
                window[i++] = BMP_GetPixelGray(inputFilter, x, y+1);
                window[i++] = BMP_GetPixelGray(inputFilter, x+1, y+1);

                int Gx = 0;
                Gx += window[0]*1;
                Gx += window[3]*2;
                Gx += window[6]*1;
                Gx += window[2]*-1;
                Gx += window[5]*-2;
                Gx += window[8]*-1;

                int Gy = 0;
                Gy += window[0]*1;
                Gy += window[1]*2;
                Gy += window[2]*1;
                Gy += window[6]*-1;
                Gy += window[7]*-2;
                Gy += window[8]*-1;


                int magnitude = 0;
                magnitude = sqrt(pow(Gx, 2) + pow(Gy, 2));

                BMP_SetPixelGray(outputFilter, x, y, magnitude);
            }
        }

        BMP_WriteFile(outputFilter, "output1.bmp");

        for (int x = 0; x < width; ++x)
        {
            for (int y = 0; y < height; ++y)
            {
                /* Get pixel's RGB values */
                BMP_GetPixelRGB(bmp1, x, y, &r, &g, &b);
                //emdebug("(%d %d %d)\n", r, g, b);
                /*Pure blue has the higher value b */
                if (b > 180 && r < 20 && g < 20)
                    //Count the number of b
                    tb++;
                /*Differentiate red from yellow*/
                else if (r > 180 && g < 10 && b < 20)
                    //Count the number of r
                    tr++;
            }
        }
        // Process image...
        checkcolor(tb, tr, &b1, &r1);
        BMP_Free(bmp);
        BMP_Free(bmp1);
        BMP_Free(inputFilter);
        BMP_Free(outputFilter);
    }
}

const uint64_t msec_to_wait = 10000; 
void delay(){
    static clock_t start = 0;

    if (start == 0)
    {
        // Set a reference point with the current clock time.
        start = clock();
    }

    // Convert ms to clock time.
    const uint64_t clocktime_to_wait = msec_to_wait * (CLOCKS_PER_SEC / 1000);

    // Retrieve the current clock time and compare it with `start`.
    // If the elapsed time exceeds our target wait time, then we're finished waiting.
    const clock_t now = clock();
    if (now - start > clocktime_to_wait)
    {
        // Finished waiting.
        // Do stuff...
        emdebug("Finished!\n");

    }
    else
    {
        emwrite(0); // Send a WAIT command, so that the Emulator is aware that our 
                    // program is still alive.
    }
}
/*LOOP SETUP*/
//ROBOT_STATE state = MOVE_TO_BASKET;
//ROBOT_STATE state = MOVE_TO_SECOND_BASKET;
ROBOT_STATE state = GRAB_TENNIS;
//ROBOT_STATE state = MOVE_TO_WHITE_TRACK;
bool move = false;
int tennis = 0;
int shooting_delay;
float power = 24.0;

void loop(void)
{
    
    emwrite(3, motor2, 0, volt_a);
    emwrite(3, motor1, 0, volt_b);
    
    if (is_data_requested)
    {
        image();
    }
    emwrite(4, 5);
    /*
    if (is_data_requested)
    {
        if (x == 2)
        {
            image();
            emdebug("%d\n", b1);
            emdebug("%d\n", r1);
        }
    }

    // Request data for a line sensor.
    if (x == 1)
    {
        emwrite(4, 5);
    }
    
    x++;
    */

    switch(state){
        case MOVE_TO_BASKET:
            //emdebug("MOVING TO BASKET\n");
            move = false;
            move = MOVE_TO_BASKET_FUNC();
            if (move){
                emdebug("PROCEEDING TO ANOTHER STATE\n");
                state = GRAB_EMPTY_BASKET;
            }
            break;
        
        case GRAB_EMPTY_BASKET:
            move = false;
            move = GRAB_EMPTY_BASKET_FUNC();
            if (move){
                emwrite(0);
                emdebug("PROCEEDING TO ANOTHER STATE\n");
                state = STOP;  
            }
            break;
        
        case MOVE_TO_SECOND_BASKET:
            emdebug("MOVING TO SECOND BASKET\n");
            move = false;
            move = MOVE_TO_SECOND_BASKET_FUNC();
            if (move){
                emdebug("PROCEEDING TO ANOTHER STATE\n");
                state = MOVE_TO_WHITE_TRACK;
            }
            break;

        case MOVE_TO_WHITE_TRACK:
            emdebug("MOVING TO WHITE TRACK\n");
            move = false;
            move = MOVE_TO_WHITE_TRACK_FUNC();
            if (move){
                emdebug("PROCEEDING TO ANOTHER STATE\n");
                state = STOP;
            }
            break;

        case GRAB_TENNIS:
            emdebug("GRABBING\n");
            emwrite(10,1);
            tennis += 1;
            if (tennis == 3){
                state = LOAD;
            }
            break;
        
        case LOAD:
            emdebug("LOAD at iter %d\n", iter);
            shooting_delay = iter;
            emdebug("SHOOTING DELAY = %d at iter = %d\n", shooting_delay, iter);
            state = IDLE;
            emwrite(20, power);
            
            break;

        case SHOOTING:
            emdebug("SHOOT at iter %d\n", iter);
            emwrite(21);
            state = STOP;
            break;

        case IDLE:
            volt_a = 0;
            volt_b = 0;
            
            emdebug("IDLE at iter %d\n", iter);
            if (iter > shooting_delay + 1){
                state = SHOOTING;
            }          
            break;
        
        case STOP:
            volt_a = 0;
            volt_b = 0;
            
    }
    
    is_data_requested = true;
    
    emwrite(0);
}
