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

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

namespace champ
{
    class TrajectoryPlanner
    {
        QuadrupedLeg *leg_;

        unsigned int total_control_points_;

        geometry::Transformation prev_foot_position_;

        float factorial_[13];
        float ref_control_points_x_[12];
        float ref_control_points_y_[12];
        float control_points_x_[12];
        float control_points_y_[12];
        
        float height_ratio_;
        float length_ratio_;

        bool run_once_;

        // float getGaitCycleCount(float target_velocity);
        void updateControlPointsHeight(float swing_height)
        {
            float new_height_ratio = swing_height / 0.15f;
            
            if(height_ratio_ != new_height_ratio)
            {
                height_ratio_ = new_height_ratio;
                for(unsigned int i = 0; i < 12; i++)
                {
                    control_points_y_[i] = -((ref_control_points_y_[i] * height_ratio_) + (0.5f * height_ratio_));
                }    
            }
        }

        void updateControlPointsLength(float step_length)
        {
            float new_length_ratio = step_length / 0.4f;
            
            if(length_ratio_ != new_length_ratio)
            {
                length_ratio_ = new_length_ratio;
                for(unsigned int i = 0; i < 12; i++)
                {
                    if(i == 0)
                        control_points_x_[i] = -step_length / 2.0f;
                    else if(i == 11)
                        control_points_x_[i] = step_length / 2.0f;
                    else
                        control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;   
                }
            }
        }

        public:
            TrajectoryPlanner(QuadrupedLeg &leg):
                leg_(&leg),    
                total_control_points_(12),
                factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
                ref_control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15},
                ref_control_points_y_{-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5},
                height_ratio_(0.0f),
                length_ratio_(0.0f),
                run_once_(false)
            {
            }

            void generate(geometry::Transformation &foot_position, float step_length, float rotation, float swing_phase_signal, float stance_phase_signal)
            {    
                updateControlPointsHeight(leg_->gait_config->swing_height);

                //ensures the prev_foot_position_ is not empty on first run
                if(!run_once_)
                {
                    run_once_ = true;
                    prev_foot_position_ = foot_position;                    
                }

                //check if there's a need to hop, otherwise nothing to do here
                if(step_length == 0.0f)
                {
                    prev_foot_position_ = foot_position;
                    leg_->gait_phase(1);

                    return;
                }

                updateControlPointsLength(step_length);

                int n = total_control_points_ - 1;
                float x = 0.0f;
                float y = 0.0f;

                if(stance_phase_signal > swing_phase_signal)
                {
                    leg_->gait_phase(1);

                    x = (step_length / 2) * (1 - (2 * stance_phase_signal));
                    y = -leg_->gait_config->stance_depth * cosf((M_PI * x) / step_length);
                }
                else if(stance_phase_signal < swing_phase_signal)
                {
                    leg_->gait_phase(0);

                    for(unsigned int i = 0; i < total_control_points_ ; i++)
                    {
                        float coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);

                        x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
                        y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
                    }
                    // x = -(step_length / 2) * (1 - (2 * swing_phase_signal));
                    // y = leg_->gait_config->swing_height * cosf((M_PI * x) / step_length);
                }
    
                foot_position.X() += x * cosf(rotation);
                foot_position.Y() += x * sinf(rotation);
                foot_position.Z() += y;

                if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f) && step_length > 0.0f)
                {
                    foot_position = prev_foot_position_;
                }

                prev_foot_position_ = foot_position;
            }
    };
}

#endif