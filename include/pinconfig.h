// motor driver pin
const int EL_left = 4;  // Enable Motor Driver Left
const int ZF_left = 12;
const int VR_left = 13; // PWM Motor Driver Left

const int EL_right = 14; // Enable Motor Driver Right
const int ZF_right = 18; // Forward or backward
const int VR_right = 19; // PWM Motor Driver Right

// tachometer or signal pin
const int left_encoder = 5;
const int right_encoder = 15;

// assign pin
const int MOTOR_PINS[6] = {EL_left, ZF_left, VR_left,
                           EL_right, ZF_right, VR_right};
const int ENCODER_PINS[2] = {left_encoder, right_encoder};