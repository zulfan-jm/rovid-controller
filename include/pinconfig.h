// motor driver pin
const int EL_left = 12;  // Enable Motor Driver Left
const int ZF_left = 14;
const int VR_left = 13; // PWM Motor Driver Left

const int EL_right = 26; // Enable Motor Driver Right
const int ZF_right = 27; // Forward or backward
const int VR_right = 25; // PWM Motor Driver Right

// tachometer or signal pin
const int left_encoder = 2;
const int right_encoder = 15;

// assign pin
const int MOTOR_PINS[6] = {EL_left, ZF_left, VR_left,
                           EL_right, ZF_right, VR_right};
const int ENCODER_PINS[2] = {left_encoder, right_encoder};