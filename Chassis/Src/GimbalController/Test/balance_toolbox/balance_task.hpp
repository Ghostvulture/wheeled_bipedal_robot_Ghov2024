#pragma once
#ifndef BALANCE_TASK_HPP
#define BALANCE_TASK_HPP

#include "kalman_filter.h"
#include "msgs.h"
#include "cstdint"
#include "arm_math.h"
/*
 *@brief: 匀加速模型卡尔曼滤波器
*/
#define t  0.001f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

#define WHEEL_RADIUS 0.095f


class cVelFusionKF {
protected:
    const float qq = 5.0f;//10
    const float rv = 0.1f;
    const float ra = 60.0f;//25.0f

    const float A_Init[9] = {1, t, t2 / 2, 0, 1, t, 0, 0, 1};
    const float Q_Init[9] = {t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq, t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq, t3 / 6 * qq,
                             t2 / 2 * qq, t * qq};
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    KalmanFilter_t KF;

    cVelFusionKF() {
        Kalman_Filter_Init(&this->KF, 3, 0, 2);//Inertia odome 3 State 2 observation
        memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
        memcpy(this->KF.F_data, A_Init, sizeof(A_Init));
        memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
        memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
        memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
    }

    void ResetKF(void) { VelFusionKF_Reset(&this->KF); }

    void UpdateKalman(float Velocity, float AccelerationX) {
        this->KF.MeasuredVector[0] = Velocity;
        this->KF.MeasuredVector[1] = AccelerationX;
        Kalman_Filter_Update(&this->KF);
    }

    float GetXhat() {
        return this->KF.xhat.pData[0];
    }

    float GetVhat() {
        return this->KF.xhat.pData[1];
    }

};



class cLinkSolver
{
    protected:

        /*雅可比矩阵*/
        float MTRTJ_mat[4]={0};
	    float JTRM_mat[4]={0};
	    float JTRMRev_mat[4]={0};

        /*腿长，单位m*/
        float L1 = 0.150f;
        float L2 = 0.270f;
        float MotorDistance = 0.150f;
        float HalfMotorDistance = MotorDistance / 2.0f;

        /*关节电机弧度*/
        float phi1 = 0.0f;
        float phi4 = 0.0f;

        /*极限值*/
        float phi1_max = PI;
        float phi4_max = PI / 2;
        float phi1_min = PI / 2;
        float phi4_min = 0.0f;

        /*倒立摆长度*/
        float PendulumLength = 0.0f;
        /*倒立摆角度*/
        float PendulumRadian = PI/2;
        /*倒立摆坐标*/
        float CoorC[2]={0.0f,0.0f};
        /*第二象限节点坐标*/
        float CoorB[2]={0.0f,0.0f};
        float U2 = 0.0f;
        /*第二象限节点坐标*/
        float CoorD[2]={0.0f,0.0f};
        float U3 = 0.0f;


    public:
	void Resolve(float phi4_radian, float psi1_radian);
	void VMCCal(float *F, float *T);
	void VMCRevCal(float *F, float *T);
    void VMCVelCal(float *phi_dot, float *phi0_dot_vt_dot);

	inline float GetPendulumLen()
	{return PendulumLength;}
	
	inline float GetPendulumRadian()
	{return PendulumRadian;}
	
	inline void GetPendulumCoor(float* Coor) 
	{Coor[0]=this->CoorC[0];Coor[1]=this->CoorC[1];}
	inline void GetCoorB(float* Coor)
	{Coor[0]=this->CoorB[0];Coor[1]=this->CoorB[1];}
	inline void GetCoorD(float*Coor)
	{Coor[0]=this->CoorD[0];Coor[1]=this->CoorD[1];}
};

#define LEG_MAX_LEN 0.270f
#define LEG_MIN_LEN 0.110f
#define LEG_START LEN 0.110f

#define LQR_MAX_LEN_CTRL 0.270f
#define LQR_MIN_LEN_CTRL 0.110f
#define LQR_LEN_RESOLUTION 0.005f
#define LEG_LEN_STEP 0.01f
#define LQR_K_NUM 27

namespace TASK_CONTROL{
    class cValUpdate {
    protected:
        float _val;
        float _path;
        bool _reached;
    public:
        cValUpdate(float default_val, float path) : _val(default_val), _path(path), _reached(false) {}
        cValUpdate() = default;
        void SetDefault(float val) {
            _val = val;
        }

        void SetPath(float path) {
            _path = path;
        }

        float GetVal() {
            return _val;
        }
        //累加，保证腿长不发生突变
        float UpdateVal(float new_val) {
            if (fabsf(new_val - _val) < 1.1f * _path) {
                _val = new_val;
                _reached = true;
            } else {
                _reached = false;
                if (new_val < _val) {
                    _val -= _path;
                } else {
                    _val += _path;
                }
            }
            return _val;
        }

        bool CheckReached() {
            return _reached;
        }
    };

    class LQR {
        protected:
            float LQRKbuf[LQR_K_NUM][12] = 
                {
                    // /* L =0.10*/
                    // {-5.730014, -0.223541, 0.304017, 0.416191, 4.461958, 1.284463, 22.654029, 1.557301, 4.371808, 6.925868, -2.978748, -1.498632},
                    /* L =0.11*/
                    {-9.752775, -0.936282, -0.197234, -0.473911, 4.456547, 1.534849, 26.375958, 4.456153, 2.761621, 5.583582, 3.055529, -0.111646},//[10 500 200 10 5000 1]
                    /* L =0.12*/
                    {-7.230003, -0.336865, 0.147707, 0.163711, 4.540392, 1.323770, 25.510221, 1.905924, 4.448657, 7.067994, -1.447229, -1.117262},
                    /* L =0.13*/
                    {-7.987448, -0.400101, 0.072447, 0.041645, 4.558602, 1.337274, 26.717555, 2.073280, 4.466499, 7.106573, -0.709828, -0.929593},
                    /* L =0.14*/
                    {-8.743461, -0.467098, -0.000436, -0.076905, 4.564354, 1.346992, 27.784353, 2.234595, 4.472136, 7.125710, 0.004271, -0.745441},
                    /* L =0.15*/
                    {-9.494066, -0.537407, -0.070718, -0.191563, 4.558873, 1.353212, 28.717412, 2.389147, 4.466765, 7.127204, 0.692889, -0.565681},
                    /* L =0.16*/
                    {-10.235900, -0.610600, -0.138247, -0.302070, 4.543370, 1.356237, 29.524438, 2.536428, 4.451575, 7.112867, 1.354541, -0.391002},
                    /* L =0.17*/
                    {-10.966167, -0.686274, -0.202932, -0.408256, 4.519018, 1.356376, 30.213688, 2.676119, 4.427715, 7.084463, 1.988318, -0.221923},
                    /* L =0.18*/
                    {-11.682598, -0.764057, -0.264727, -0.510029, 4.486929, 1.353929, 30.793685, 2.808048, 4.396274, 7.043684, 2.593786, -0.058821},
                    /* L =0.19*/
                    {-12.383386, -0.843605, -0.323628, -0.607361, 4.448144, 1.349185, 31.272994, 2.932173, 4.358273, 6.992117, 3.170897, 0.098056},
                    /* L =0.20*/
                    {-13.067143, -0.924605, -0.379662, -0.700274, 4.403623, 1.342417, 31.660050, 3.048551, 4.314652, 6.931241, 3.719914, 0.248566},
                    /* L =0.21*/
                    {-13.732844, -1.006775, -0.432881, -0.788833, 4.354248, 1.333881, 31.963026, 3.157323, 4.266274, 6.862410, 4.241349, 0.392656},
                    /* L =0.22*/
                    {-14.379778, -1.089862, -0.483356, -0.873136, 4.300814, 1.323813, 32.189739, 3.258689, 4.213920, 6.786863, 4.735902, 0.530344},
                    /* L =0.23*/
                    {-15.007503, -1.173642, -0.531174, -0.953303, 4.244040, 1.312427, 32.347591, 3.352897, 4.158293, 6.705717, 5.204421, 0.661710},
                    /* L =0.24*/
                    {-15.615803, -1.257915, -0.576432, -1.029474, 4.184567, 1.299921, 32.443527, 3.440229, 4.100021, 6.619973, 5.647855, 0.786880},
                    /* L =0.25*/
                    {-16.204653, -1.342507, -0.619234, -1.101802, 4.122964, 1.286471, 32.484014, 3.520991, 4.039663, 6.530528, 6.067226, 0.906015},
                    /* L =0.26*/
                    {-16.774180, -1.427267, -0.659688, -1.170448, 4.059735, 1.272238, 32.475040, 3.595502, 3.977711, 6.438175, 6.463600, 1.019302},
                    /* L =0.27*/
                    {-17.324640, -1.512062, -0.697907, -1.235578, 3.995321, 1.257362, 32.422114, 3.664087, 3.914599, 6.343614, 6.838064, 1.126951},
                    /* L =0.28*/
                    {-17.856388, -1.596780, -0.734001, -1.297359, 3.930110, 1.241972, 32.330285, 3.727075, 3.850705, 6.247458, 7.191710, 1.229181},
                    /* L =0.29*/
                    {-18.369857, -1.681323, -0.768080, -1.355960, 3.864435, 1.226178, 32.204161, 3.784789, 3.786358, 6.150246, 7.525620, 1.326222},
                    /* L =0.30*/
                    {-18.865542, -1.765609, -0.800254, -1.411544, 3.798588, 1.210080, 32.047930, 3.837546, 3.721841, 6.052444, 7.840854, 1.418307},
                    /* L =0.31*/
                    {-19.343978, -1.849568, -0.830626, -1.464272, 3.732818, 1.193765, 31.865388, 3.885653, 3.657400, 5.954454, 8.138444, 1.505668},
                    /* L =0.32*/
                    {-19.805734, -1.933144, -0.859300, -1.514301, 3.667338, 1.177309, 31.659968, 3.929408, 3.593242, 5.856624, 8.419388, 1.588537},
                    /* L =0.33*/
                    {-20.251397, -2.016288, -0.886373, -1.561781, 3.602328, 1.160777, 31.434761, 3.969094, 3.529546, 5.759250, 8.684643, 1.667139},
                    /* L =0.34*/
                    {-20.681564, -2.098962, -0.911937, -1.606856, 3.537942, 1.144227, 31.192550, 4.004982, 3.466461, 5.662585, 8.935126, 1.741695},
                    /* L =0.35*/
                    {-21.096837, -2.181135, -0.936084, -1.649664, 3.474307, 1.127709, 30.935832, 4.037327, 3.404112, 5.566842, 9.171708, 1.812419},
                    /* L =0.36*/
                    {-21.497813, -2.262782, -0.958895, -1.690337, 3.411529, 1.111265, 30.666845, 4.066374, 3.342602, 5.472198, 9.395219, 1.879516},
                    /* L =0.37*/
                    {-21.885082, -2.343885, -0.980453, -1.728998, 3.349694, 1.094932, 30.387587, 4.092351, 3.282016, 5.378803, 9.606441, 1.943183}

                };
        
            float LQROutBuf[2] = {0};
            float LQRXerrorBuf[6] = {0};
            float MatLQRNegK_fly[12] = {0};

            arm_matrix_instance_f32 *LQRXRefX;
            arm_matrix_instance_f32 *LQRXObsX;

            arm_matrix_instance_f32 MatLQRNegK = {2, 6, (float *) LQRKbuf[0]};
            arm_matrix_instance_f32 MatLQRErrX = {6, 1, LQRXerrorBuf};
            arm_matrix_instance_f32 MatLQROutU = {2, 1, LQROutBuf};

        public:

            /*[T;Tp] = k(X_obs - X_ref)*/
            void LQRCal(float *Tout)
            {
                //calculate (X_obs - X_ref)
                arm_mat_sub_f32(this->LQRXObsX, this->LQRXRefX, &this->MatLQRErrX);
                //calculate U = -K(X_obs - X_ref)
                arm_mat_mult_f32(&this->MatLQRNegK, &this->MatLQRErrX, &this->MatLQROutU);
                //return value
                Tout[0] = this->LQROutBuf[0];
                Tout[1] = this->LQROutBuf[1];

            }

            /*根据腿长更新使用的矩阵k，小板凳模式只使用第一个K
                @todo1:加入变腿长
                @todo2:加入支持力解算
            */
            void refreshLQRK(float LegLenth)
            {
                LegLenth = (LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : LegLenth;
                LegLenth = (LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : LegLenth;
                volatile uint8_t ID = roundf((LegLenth - LQR_MIN_LEN_CTRL) / LQR_LEN_RESOLUTION);

                this->MatLQRNegK.pData = (float *) LQRKbuf[ID];

            }

            void InitMatX(arm_matrix_instance_f32 *pMatXRef, arm_matrix_instance_f32 *pMatXObs) {
                this->LQRXRefX = pMatXRef;
                this->LQRXObsX = pMatXObs;
            }

    };
}


#endif // BALANCE_TASK_HPP