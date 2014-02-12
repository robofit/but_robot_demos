
#include "RoadCheck.h"

RoadCheck::RoadCheck() {
}

RoadCheck::RoadCheck(int frame_length_angle, float g, float gth, int frame_length_road) {
    this->x_frame = new float[frame_length_angle];
    this->y_frame = new float[frame_length_angle];
    this->z_frame = new float[frame_length_angle];

    this->v_frame = new float[frame_length_angle];

    this->angle_x = 0;
    this->angle_y = 0;
    this->angle_z = 0;

    this->length = frame_length_angle;
    this->count = 0;
    this->frame_full = false;
    this->end = 0;

    this->g = g;
    this->gth = gth;

    b_timing = new float[frame_length_road];

    double ssum = 0;
    for (int i = 0; i < frame_length_road; i++) {
        ssum += frame_length_road + 1 - i;
    }


    for (int i = 0; i < frame_length_road; i++) {
        // b_timing[i] = 41 - i;
        b_timing[i] = (frame_length_road + 1 - i) / ssum;
    }

    b_mean = new float[15];
    for (int i = 0; i < 15; i++) {
        b_mean[i] = 1;
    }
    this->timing = new Filter(b_timing, frame_length_road);
    this->mean = new Filter(b_mean, 15);
    med_i = 0;
    med = new float[3];
}

RoadCheck::~RoadCheck() {
    delete [] x_frame;
    delete [] y_frame;
    delete [] z_frame;
    delete [] b_timing;
    delete [] b_mean;
    delete timing;
    delete mean;
    delete [] v_frame;
    delete [] med;
}

void RoadCheck::clearAll() {
    this->count = 0;
    this->frame_full = false;
    this->end = 0;
}

float RoadCheck::addSample(float x, float y, float z) {
    x_frame[this->end] = x;
    y_frame[this->end] = y;
    z_frame[this->end] = z;

    float v = fabs(sqrt(x * x + y * y + z * z) - this->g);

    if (v <= gth) {
        v = 0.0f;
    }
    med[med_i] = v;
    med_i = (med_i + 1) % 3;
    v = median(med[0], med[1], med[2]);
    v = mean->filtering(timing->filtering(v));

    if (!this->frame_full) {
        this->count++;
        if (this->count> this->length) {
            this->frame_full = true;
        }
    }
    
    this->end = (this->end + 1) % this->length;
    return v;
}

bool RoadCheck::countAngles() {
    if (!this->frame_full) {
        return false;
    }
    float mu_x = 0;
    float mu_y = 0;
    float mu_z = 0;
    float mu_v = 0;
    for (int i = 0; i<this->length; i++) {
        mu_x += x_frame[i];
        mu_y += y_frame[i];
        mu_z += z_frame[i];
        mu_v += v_frame[i];
    }
    mu_x /= this->length;
    mu_y /= this->length;
    mu_z /= this->length;
    mu_v /= this->length;
    this->angle_x = atan(mu_x / sqrt(mu_y * mu_y + mu_z * mu_z));
    this->angle_y = atan(mu_y / sqrt(mu_x * mu_x + mu_z * mu_z));
    this->angle_z = atan(mu_z / sqrt(mu_y * mu_y + mu_x * mu_x));
    this->v = mu_v;
    return true;
}

float RoadCheck::getV() {
    return this->v;
}

float RoadCheck::getAngleX() {
    return this->angle_x;
}

float RoadCheck::getAngleY() {
    return this->angle_y;
}

float RoadCheck::getAngleZ() {
    return this->angle_z;
}

float RoadCheck::median(float a, float b, float c) {
    if (a > b) {
        if (b > c) {
            return b;
        } else if (a > c) {
            return c;
        } else {
            return a;
        }
    } else {
        if (a > c) {
            return a;
        } else if (b > c) {
            return c;
        } else {
            return b;
        }
    }

}


