/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef QUADRUPED_JOINT_H
#define QUADRUPED_JOINT_H

#include <quadruped_base/quadruped_components.h>

namespace champ
{
    class Joint
    {
        // float translation_.x; 
        // float translation_.y; 
        // float translation_.z; 

        // float rotation_.roll; 
        // float rotation_.pitch; 
        // float rotation_.yaw;

        float theta_;

        champ::Point translation_;
        champ::Euler rotation_;

        public:
            Joint():
                theta_(0.0)
            {
            }
            
            Joint(float pos_x, float pos_y, float pos_z, 
                  float or_r,  float or_p,  float or_y):
                theta_(0.0)
            { 
                translation_.x = pos_x;
                translation_.y = pos_y;
                translation_.z = pos_z;
                rotation_.roll = or_r;
                rotation_.pitch = or_p;
                rotation_.yaw = or_y;
            } 

            float theta()
            { 
                return theta_; 
            }

            void theta(float angle)
            { 
                theta_ = angle; 
            }

            void setTranslation(float x, float y, float z)
            {
                translation_.x = x;
                translation_.y = y;
                translation_.z = z;
            }

            void setRotation(float roll, float pitch, float yaw)
            {
                rotation_.roll = roll;
                rotation_.pitch = pitch;
                rotation_.yaw = yaw;
            }

            void setOrigin(float x, float y, float z,
                    float roll, float pitch, float yaw)
            {
                translation_.x = x;
                translation_.y = y;
                translation_.z = z;
                rotation_.roll = roll;
                rotation_.pitch = pitch;
                rotation_.yaw = yaw;
            }

            const float x() const
            {
                return translation_.x;
            }

            const float y() const
            {
                return translation_.y;
            }

            const float z() const
            {
                return translation_.z;
            }

            const float roll() const
            {
                return rotation_.roll;
            }

            const float pitch() const
            {
                return rotation_.pitch;
            }

            const float yaw() const
            {
                return rotation_.yaw;
            }

            // void x(float x)
            // {
            //     translation_.x = x;
            // }

            // void y(float y)
            // {
            //     translation_.y = y;
            // }

            // void z(float z)
            // {
            //     translation_.z = z;
            // }

            // void roll(float roll)
            // {
            //     rotation_.roll = roll;
            // }

            // void pitch(float pitch)
            // {
            //     rotation_.pitch = pitch;
            // }

            // void yaw(float yaw)
            // {
            //     rotation_.yaw = yaw;
            // }
    };
}
#endif