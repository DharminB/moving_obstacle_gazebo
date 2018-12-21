/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified by : Dharmin B.
*/
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedObject : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;
        
        double animation_time = 0.0;
        int number_of_frames = 0; 
        bool loop = false; 
        std::vector<double> key_frames;
        std::vector<double> x_positions;
        std::vector<double> y_positions;
        std::vector<double> orientations;
        
        // get arguments
        if (_sdf->HasElement("animation_time")){
            animation_time = _sdf->Get<double>("animation_time");
        }
        if (_sdf->HasElement("loop")){
            loop = _sdf->Get<bool>("loop");
        }
        if (_sdf->HasElement("number_of_frames")){
            number_of_frames = _sdf->Get<int>("number_of_frames");
        }
        key_frames = this->extract_vector_from_string_parameter("key_frames", _sdf);
        x_positions = this->extract_vector_from_string_parameter("x_positions", _sdf);
        y_positions = this->extract_vector_from_string_parameter("y_positions", _sdf);
        orientations = this->extract_vector_from_string_parameter("orientations", _sdf);

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              new gazebo::common::PoseAnimation("animation", animation_time, loop));

        gazebo::common::PoseKeyFrame *key;

        // check if number of frames is same as the number of key frame paramters
        if (number_of_frames == key_frames.size() && 
                number_of_frames == x_positions.size() &&
                number_of_frames == y_positions.size() &&
                number_of_frames == orientations.size()){
            // set key frames in the animation
            for (int i = 0; i < number_of_frames; ++i) {
                key = anim->CreateKeyFrame(key_frames.at(i));
                key->Translation(ignition::math::Vector3d(x_positions.at(i), y_positions.at(i), 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, orientations.at(i)));
            }
            if (loop) {
                key = anim->CreateKeyFrame(animation_time);
                key->Translation(ignition::math::Vector3d(x_positions.at(0), y_positions.at(0), 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, orientations.at(0)));
            }
            // set the animation
            _parent->SetAnimation(anim);
        }
        else{
            std::cout << "Moving obstacle plugin received erroneous parameters" << std::endl;
        }
    }
          
    // return a vector of double after reading a parameter 
    private: std::vector<double> extract_vector_from_string_parameter(std::string param_name, sdf::ElementPtr _sdf)
    {
        std::string vector_string; 
        std::vector<double> parameter;
        if (_sdf->HasElement(param_name)){
            vector_string = _sdf->Get<std::string>(param_name);
        }
        else {
            parameter.push_back(0);
            return parameter;
        }
        
        // split space separated arguments
        std::istringstream iss(vector_string);
        std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                         std::istream_iterator<std::string>());
        for (std::string i : results) {
            parameter.push_back (std::stod(i));
        }
        return parameter;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedObject)
}
