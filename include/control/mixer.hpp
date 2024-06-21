enum MotorGeometry {
    QUAD_X,
    QUAD_PLUS,
};

class Mixer {
public:
    Mixer(MotorGeometry geometry);
    void mix(float roll, float pitch, float yaw, float throttle, float* motor_out);
};