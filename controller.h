class Ctrl {

    public:
        Ctrl();
        float ref;
        float error, error_last;
        float volt;
        float theta, theta_last;
        float dt;
        float w0, w1, w2, lp_w;

        void init();
        float PID();
        float get_omega();
        float low_pass_filter();
        void omega_processor();

};