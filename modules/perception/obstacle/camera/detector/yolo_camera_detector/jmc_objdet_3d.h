/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_3D_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_3D_DETECTOR_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "caffe/caffe.hpp"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_object_detector.h"

#define Y3D_TYPE 0 //0: Y3D; 1: Y5D; 2: Y3D_BY5

namespace jmc_auto {
namespace perception {

    #if (Y3D_TYPE == 0)

    //static std::string prototxt_path = "./jmc_model/jmc_v1.0.0.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v1.0.0.caffemodel";
    //static std::string prototxt_path = "./jmc_model/jmc_v1.3.0-210726.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v1.3.0-210726.caffemodel";

    //static std::string prototxt_path = "./jmc_model/jmc_v1.0.0_210828.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v1.0.0_210828.caffemodel";

    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v1.0.0_210830.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v1.0.0_210830.caffemodel";

    #elif (Y3D_TYPE == 2)
    //static std::string prototxt_path = "./jmc_model/jmc_v1.5.0-210823.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v1.5.0-210823.caffemodel";
    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v1.5.0-210824.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v1.5.0-210824.caffemodel";

    #elif (Y3D_TYPE == 1)
    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.9_kt.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.9_kt.caffemodel";
    #endif

    typedef struct _t_Bbox3dPts
    {
        Eigen::Vector3f left;
        Eigen::Vector3f top;
        Eigen::Vector3f right;
        Eigen::Vector3f bottom;
    }tBbox3dPts;

    class JmcObjdet3D : public jmc_auto::perception::JmcObjectDetector
    {
        public:
        JmcObjdet3D()
        {
            input_width_ = 640;
            #if ((Y3D_TYPE == 0) || (Y3D_TYPE == 2))
            input_height_ = 224;
            //input_height_ = 640;
            #elif (Y3D_TYPE == 1)
            input_height_ = 640;
            #endif
            num_class_ = 8;
            if(data_)
            {
                delete[] data_;
                data_=nullptr;
            }
            data_ = new float[3 * input_width_ * input_height_];

            SetConfigFilePath(prototxt_path,weight_file,true);
            #if ((Y3D_TYPE == 0) || (Y3D_TYPE == 2))
            //names = ['Car', 'Van', 'Truck', 'Pedestrian','Person_sitting', 'Cyclist', 'Tram', 'Misc']
            float dim_ref[3*8] = {
                1.52607842, 1.62858147, 3.88396124, //# Car h, w, l
                2.20649159, 1.90197734, 5.07812564, //# Van h, w, l
                3.25207685,  2.58505032, 10.10703568,//# Truck h, w, l
                1.76067766, 0.6602296 , 0.84220464,//# Pedestrian h, w, l
                1.27538462, 0.59506787, 0.80180995,//# Person_sitting h, w, l
                1.73712792, 0.59677122, 1.76338868,//# Cyclist h, w, l
                3.52905882,  2.54368627, 16.09707843,//# Tram h, w, l
                1.9074177, 1.51386831, 3.57683128}; //# Misc h, w, l

            #elif(Y3D_TYPE == 1)
            float dim_ref[3*8] = {1.76067766, 0.6602296 , 0.84220464,//# Pedestrian h, w, l
                1.52607842, 1.62858147, 3.88396124, //# Car h, w, l
                1.73712792, 0.59677122, 1.76338868,//# Cyclist h, w, l
                2.20649159, 1.90197734, 5.07812564, //# Van h, w, l
                3.25207685,  2.58505032, 10.10703568,//# Truck h, w, l
                1.27538462, 0.59506787, 0.80180995,//# Person_sitting h, w, l
                3.52905882,  2.54368627, 16.09707843,//# Tram h, w, l
                1.9074177, 1.51386831, 3.57683128}; //# Misc h, w, l
            #endif
            memcpy(dim_ref_,dim_ref,sizeof(dim_ref));
            #if 1
            #if 0
            float inter_k[9]={1293.5,0.0,960
                              ,0.0,1293.5,546
                              ,0.0,0.0,1.0};
            #else
            float inter_k[9]={862.5,0.0,610
                              ,0.0,862.5,360
                              ,0.0,0.0,1.0};
            #endif
            #else
            float inter_k[9]={721.5,0.0,609.6
                              ,0.0,721.5,172.9
                              ,0.0,0.0,1.0};
            #endif

            memcpy(inter_k_,inter_k,sizeof(inter_k_));
        }

        virtual ~JmcObjdet3D()
        {
          if(data_)
          {
            delete[] data_;
            data_=nullptr;
          }
        }

        private:
            int num_3d_info_{9};
            float dim_ref_[3*8];
            float inter_k_[9];

        public:

        bool SetInterK(float *inter_k)
        {
            bool ret = false;
            if(inter_k)
            {
                memcpy(inter_k_,inter_k,sizeof(inter_k_));
                ret=true;
            }
            return ret;
        }

        bool cvtToVisualObject(std::vector<Boundingbox> bboxes,std::vector<std::shared_ptr<VisualObject>> *objects)
        {
            bool ret = true;
            if(objects)
            {
                if(bboxes.size() > 0)
                {
                    int id = 0;
                    for(int i = 0; i < (int)bboxes.size(); i++)
                    {
                        ObjectType type = cvtToJmcobj(bboxes[i].cid);
                        int max_cls = static_cast<int>(ObjectType::MAX_OBJECT_TYPE);
                        if(static_cast<int>(type) < max_cls)
                        {
                            std::shared_ptr<VisualObject> obj(new VisualObject);
                            obj->type = type;
                            obj->id = id++;
                            obj->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),0.0f);
                            for (int k = 0; k < max_cls; ++k)
                            {
                                obj->type_probs[k] = 0.0;
                                if(k == static_cast<int>(obj->type))
                                {
                                    obj->type_probs[k] = bboxes[i].score;
                                }
                            }
                            CalCenters(bboxes[i]);
                            CalConners(bboxes[i]);
                            obj->front_upper_left.x() = bboxes[i].front[3].x;
                            obj->front_upper_left.y() = bboxes[i].front[3].y;
                            obj->front_lower_right.x() = bboxes[i].rear[0].x;
                            obj->front_lower_right.y() = bboxes[i].rear[0].y;

                            obj->back_upper_left.x() = bboxes[i].front[2].x;
                            obj->back_upper_left.y() = bboxes[i].front[2].y;
                            obj->back_lower_right.x() = bboxes[i].rear[1].x;
                            obj->back_lower_right.y() = bboxes[i].rear[1].y;
                            for(int k = 0; k < 4; k++)
                            {
                                obj->pts8.push_back(bboxes[i].front[k].x);
                                obj->pts8.push_back(bboxes[i].front[k].y);
                                obj->pts8.push_back(bboxes[i].rear[k].x);
                                obj->pts8.push_back(bboxes[i].rear[k].y);
                            }

                            obj->upper_left[0] = bboxes[i].xmin;
                            obj->upper_left[1] = bboxes[i].ymin;
                            obj->lower_right[0] = bboxes[i].xmax;
                            obj->lower_right[1] = bboxes[i].ymax;
                            obj->alpha = bboxes[i].alpha;
                            obj->theta = bboxes[i].ray;
                            obj->width = bboxes[i].dim_w;
                            obj->height = bboxes[i].dim_h;
                            obj->length = bboxes[i].dim_l;
                            obj->center.x() = bboxes[i].center_x;
                            obj->center.y() = bboxes[i].center_y;
                            obj->center.z() = bboxes[i].center_z;

                            objects->push_back(obj);
                        }
                    }
                }
            }
            else
            {
                std::cout << "'objects' is a null pointer."<<std::endl;
            }
            return ret;
        }

        Eigen::Matrix3f GetRotationMat(float fyaw,float fpitch = 0.0,float froll = 0.0)
        {
            Eigen::Matrix3f ext_Rx,ext_Ry,ext_Rz,ext_R;
            ext_Rx << 1,0,0,0,cos(fpitch),-sin(fpitch),0,sin(fpitch),cos(fpitch);
            ext_Ry << cos(fyaw),0,sin(fyaw),0,1,0,-sin(fyaw),0,cos(fyaw);
            ext_Rz << cos(froll),-sin(froll),0,sin(froll),cos(froll),0,0,0,1;

            ext_R = ext_Rx * ext_Ry * ext_Rz;
            return ext_R;
        }

        float deg2rad(float deg)
        {
            return deg * 3.1415926 / 180.0;
        }

        bool CalConners(Boundingbox &bbox)
        {
            bool ret = true;
            float dz = bbox.dim_w / 2.0;
            float dy = bbox.dim_h / 2.0;
            float dx = bbox.dim_l / 2.0;
            float orient = bbox.alpha + bbox.ray;
            std::vector<Eigen::Vector3f> conners;
            //Eigen::Matrix3f proj_matrix = Eigen::Map<Eigen::Matrix3f>(inter_k_,3,3);
            Eigen::Matrix3f proj_matrix;
            proj_matrix << inter_k_[0],inter_k_[1],inter_k_[2],inter_k_[3],inter_k_[4],inter_k_[5],inter_k_[6],inter_k_[7],inter_k_[8];

            Eigen::Matrix3f R = GetRotationMat(orient);
            Eigen::Vector3f center(bbox.center_x,bbox.center_y,bbox.center_z);
            float index[2]={-1.0 ,1.0};
            for(auto i : index){
                for(auto j : index){
                    for(auto k : index){
                        Eigen::Vector3f conner(dx * i, dy * j, dz * k);
                        Eigen::Vector3f world = Eigen::Vector3f::Zero();
                        world = R * conner + center;
                        //Eigen::Vector3f wd((world.x()/world.z()+0.0001),(world.y()/world.z()+0.0001),1.0);
                        //Eigen::Vector3f wd(world.x(),world.y(),1.0);
                        //Eigen::Vector3f pt = proj_matrix * wd;
                        Eigen::Vector3f pt = proj_matrix * world;
                        pt = pt/pt.z();
                        conners.push_back(pt);
                    }
                }
            }
            if(conners.size() == 8)
            {
                bbox.front[0].x = conners[0].x();
                bbox.front[0].y = conners[0].y();
                bbox.front[1].x = conners[4].x();
                bbox.front[1].y = conners[4].y();
                bbox.front[2].x = conners[6].x();
                bbox.front[2].y = conners[6].y();
                bbox.front[3].x = conners[2].x();
                bbox.front[3].y = conners[2].y();

                bbox.rear[0].x = conners[1].x();
                bbox.rear[0].y = conners[1].y();
                bbox.rear[1].x = conners[5].x();
                bbox.rear[1].y = conners[5].y();
                bbox.rear[2].x = conners[7].x();
                bbox.rear[2].y = conners[7].y();
                bbox.rear[3].x = conners[3].x();
                bbox.rear[3].y = conners[3].y();
            }
            else
            {
                ret = false;
            }

            return ret;
        }

        bool CalCenters(Boundingbox &bbox)
        {
            bool ret = false;
            float box_corners[4];
            box_corners[0] = bbox.xmin;
            box_corners[1] = bbox.ymin;
            box_corners[2] = bbox.xmax;
            box_corners[3] = bbox.ymax;

            float dz = bbox.dim_w / 2.0;
            float dy = bbox.dim_h / 2.0;
            float dx = bbox.dim_l / 2.0;

            float alpha = bbox.alpha;
            float center_x = (bbox.xmin + bbox.xmax)/2.0;
            float theta_ray = atan((center_x - inter_k_[2]) / inter_k_[0]);

            float orient = alpha + theta_ray;
            //Eigen::Matrix3f proj_matrix = Eigen::Map<Eigen::Matrix3f>(inter_k_,3,3);
            Eigen::MatrixXf proj_matrix_3X4(3, 4);
            //Eigen::Matrix<float, 3, 4> proj_matrix_3X4;
            proj_matrix_3X4 << inter_k_[0],inter_k_[1],inter_k_[2],0.0,inter_k_[3],inter_k_[4],inter_k_[5],0.0,inter_k_[6],inter_k_[7],inter_k_[8],0.0;
            //proj_matrix << inter_k_;
            bbox.ray = theta_ray;

            Eigen::Matrix3f R = GetRotationMat(orient);

            std::vector<Eigen::Vector3f> left_constraints;
            std::vector<Eigen::Vector3f> right_constraints;
            std::vector<Eigen::Vector3f> top_constraints;
            std::vector<Eigen::Vector3f> bottom_constraints;

            float left_mult = 1.0;
            float right_mult = -1.0;
            float switch_mult = -1.0;

            if(alpha > 0.0)
            {
                switch_mult = 1.0;
            }

            if (alpha < deg2rad(92.0) && alpha > deg2rad(88))
            {
                left_mult = 1.0;
                right_mult = 1.0;
            }
            else if (alpha < deg2rad(-88.0) && alpha > deg2rad(-92.0))
            {
                left_mult = -1.0;
                right_mult = -1.0;
            }
            else if (alpha < deg2rad(90.0) && alpha > deg2rad(-90.0))
            {
                left_mult = -1.0;
                right_mult = 1.0;
            }
            float index[2]={-1.0 ,1.0};

            for(float i : index)
            {
                Eigen::Vector3f left;
                Eigen::Vector3f right;
                left << left_mult * dx, i * dy, -switch_mult * dz;
                right << right_mult * dx, i * dy, switch_mult * dz;
                left_constraints.push_back(left);
                right_constraints.push_back(right);
                for(float j : index)
                {
                    Eigen::Vector3f top;
                    Eigen::Vector3f bottom;
                    top << i * dx, -dy, j * dz;
                    bottom << i * dx, dy, j * dz;
                    top_constraints.push_back(top);
                    bottom_constraints.push_back(bottom);
                }
            }
            float loss = std::numeric_limits<float>::max();
            Eigen::MatrixXf result;
            for(auto &left : left_constraints){
                for(auto &top : top_constraints){
                    for(auto &right : right_constraints){
                        for(auto &bottom : bottom_constraints){
                            std::vector<Eigen::Vector3f> X_array;
                            X_array.push_back(left);
                            X_array.push_back(top);
                            X_array.push_back(right);
                            X_array.push_back(bottom);
                            int indicies[4] = {0, 1, 0, 1};
                            Eigen::Matrix<float, 4, 3> A_43;
                            Eigen::Matrix<float, 4, 1> b_41;
                            for(int row = 0; row < 4; row++)
                            {
                                int index = indicies[row];
                                Eigen::Vector3f RX = Eigen::Vector3f::Zero();
                                RX = R * X_array[row];
                                Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
                                //M.eye();
                                M<<1,0,0,RX.x(),0,1,0,RX.y(),0,0,1,RX.z(),0,0,0,1;
                                //Eigen::Matrix4f MM = Eigen::Matrix4f::Zero();
                                //std::cout << M << std::endl;
                                Eigen::MatrixXf M_34(3, 4);
                                //Eigen::Matrix<float, 3, 4> M_34;
                                M_34 = proj_matrix_3X4  *  M;
                                //std::cout << M_34 << std::endl;
                                //box_corners[row]
                                //A_43.row(row) =
                                A_43(row,0) = M_34(index,0) - box_corners[row]*M(2,0);
                                A_43(row,1) = M_34(index,1) - box_corners[row]*M(2,1);
                                A_43(row,2) = M_34(index,2) - box_corners[row]*M(2,2);
                                b_41(row,0) = box_corners[row]*M(2,3) - M_34(index,3);
                            }
                            //std::cout << A_43 << std::endl;
                            //std::cout << b_41 << std::endl;
                            Eigen::MatrixXf X_Res = A_43.colPivHouseholderQr().solve(b_41);
                            Eigen::MatrixXf pre_bbox = A_43 * X_Res;
                            float ll = (pre_bbox(0,0) - b_41(0,0))*(pre_bbox(0,0) - b_41(0,0))
                                    +(pre_bbox(1,0) - b_41(1,0))*(pre_bbox(1,0) - b_41(1,0))
                                    +(pre_bbox(2,0) - b_41(2,0))*(pre_bbox(2,0) - b_41(2,0))
                                    +(pre_bbox(3,0) - b_41(3,0))*(pre_bbox(3,0) - b_41(3,0));
                            if(ll < loss)
                            {
                                result = X_Res;
                                loss = ll;
                                ret = true;
                            }
                            //std::cout << X_Res << std::endl;
                            //std::cout << "loss: "<<loss<<std::endl;
                        }
                    }
                }
            }
            if(ret)
            {
                bbox.center_x = result(0,0);
                bbox.center_y = result(1,0);
                bbox.center_z = result(2,0);
            }
            return ret;
        }

        protected:
        std::vector<Anchor> InitAnchors()
        {
            std::vector<Anchor> anchors;
            Anchor anchor;
        #if (Y3D_TYPE == 0)
            //[106,68,  175,105,  262,140]  # P5/32
            //[44,32,  75,37,  46,95]  # P4/16
            //[19,16,  15,40,  32,22]  # P3/8

            anchor.width = 106;
            anchor.height = 68;
            anchors.emplace_back(anchor);
            anchor.width = 175;
            anchor.height = 105;
            anchors.emplace_back(anchor);
            anchor.width = 262;
            anchor.height = 140;
            anchors.emplace_back(anchor);
            anchor.width = 44;
            anchor.height = 32;
            anchors.emplace_back(anchor);
            anchor.width = 75;
            anchor.height = 37;
            anchors.emplace_back(anchor);
            anchor.width = 46;
            anchor.height = 95;
            anchors.emplace_back(anchor);
            anchor.width = 19;
            anchor.height = 16;
            anchors.emplace_back(anchor);
            anchor.width = 15;
            anchor.height = 40;
            anchors.emplace_back(anchor);
            anchor.width = 32;
            anchor.height = 22;
            anchors.emplace_back(anchor);
        #elif ((Y3D_TYPE == 1) || (Y3D_TYPE == 2))
        //#else
            // 10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90,  156,198,  373,326
            anchor.width = 10;
            anchor.height = 13;
            anchors.emplace_back(anchor);
            anchor.width = 16;
            anchor.height = 30;
            anchors.emplace_back(anchor);
            anchor.width = 32;
            anchor.height = 23;
            anchors.emplace_back(anchor);
            anchor.width = 30;
            anchor.height = 61;
            anchors.emplace_back(anchor);
            anchor.width = 62;
            anchor.height = 45;
            anchors.emplace_back(anchor);
            anchor.width = 59;
            anchor.height = 119;
            anchors.emplace_back(anchor);
            anchor.width = 116;
            anchor.height = 90;
            anchors.emplace_back(anchor);
            anchor.width = 156;
            anchor.height = 198;
            anchors.emplace_back(anchor);
            anchor.width = 373;
            anchor.height = 326;
            anchors.emplace_back(anchor);
        #endif
            return anchors;
        }


        void PostProcessParall(const int height, const int width, int scale_idx, float postThres,
                      float * origin_output, std::vector<int> Strides, std::vector<Anchor> Anchors, std::vector<Boundingbox> *bboxes)
        {
            Boundingbox bbox;
            float cx, cy, w_b, h_b, score;
            int cid;
            const float *ptr = (float *)origin_output;
            //int cnt=0;
            memset(&bbox,0x0,sizeof(Boundingbox));
            for(unsigned long a = 0; a < 3; ++a){
                for(unsigned long h = 0; h < (unsigned long)height; ++h){
                    for(unsigned long w=0; w <(unsigned long)width; ++w){
                        //std::cout <<" x["<<cnt++<<"]: " << ptr[0] << std::endl;
                        //if(ptr[4] > 0.4)
                        {
                            const float *cls_ptr =  ptr + 5;
                            cid = argmax(cls_ptr, cls_ptr+num_class_);
                            //score = ptr[4] * cls_ptr[cid];
                            score = sigmoid(ptr[4]) * sigmoid(cls_ptr[cid]);
                            if(score>=postThres){
                            //if(score>=0.1){
                                #if 0
                                cx = ptr[0];
                                cy = ptr[1];
                                w_b = ptr[2];
                                h_b = ptr[3];
                                #else
                                cx = (sigmoid(ptr[0]) * 2.f - 0.5f + static_cast<float>(w)) * static_cast<float>(Strides[scale_idx]);
                                cy = (sigmoid(ptr[1]) * 2.f - 0.5f + static_cast<float>(h)) * static_cast<float>(Strides[scale_idx]);
                                w_b = powf(sigmoid(ptr[2]) * 2.f, 2)* Anchors[scale_idx * 3 + a].width;
                                h_b = powf(sigmoid(ptr[3]) * 2.f, 2)* Anchors[scale_idx * 3 + a].height;
                                #endif
                                bbox.xmin = clip(cx - w_b / 2, 0.F, static_cast<float>(input_width_ - 1));
                                bbox.ymin = clip(cy - h_b / 2, 0.f, static_cast<float>(input_height_ - 1));
                                bbox.xmax = clip(cx + w_b / 2, 0.f, static_cast<float>(input_width_ - 1));
                                bbox.ymax = clip(cy + h_b / 2, 0.f, static_cast<float>(input_height_ - 1));
                                bbox.score = score;
                                bbox.cid = cid;
                                bbox.with_3d = 1;
                                int bin_num = 2;
                                const float *info_3d_ptr = cls_ptr + num_class_;
                                int ori_conf = argmax(info_3d_ptr,info_3d_ptr+bin_num);
                                #if ((Y3D_TYPE == 0) || (Y3D_TYPE == 2))
                                float sin_ap = *(info_3d_ptr + (1 + ori_conf)* bin_num + 1);
                                float cos_ap = *(info_3d_ptr + (1 + ori_conf)* bin_num);
                                #elif (Y3D_TYPE == 1)
                                float sin_ap = *(info_3d_ptr + (1 + ori_conf)* bin_num);
                                float cos_ap = *(info_3d_ptr + (1 + ori_conf)* bin_num + 1);
                                #endif
                                bbox.alpha = atan2(sin_ap,cos_ap);
                                #if ((Y3D_TYPE == 0) || (Y3D_TYPE == 2))
                                if(ori_conf == 0)
                                {
                                    bbox.alpha += 3.1415926 / 2.0;
                                }
                                else if(ori_conf == 1)
                                {
                                    bbox.alpha += 3.1415926 * 3.0/ 2.0;
                                }
                                bbox.alpha -= 3.1415926;
                                #else
                                if(ori_conf == 0)
                                {
                                    bbox.alpha += 3.1415926 / 2.0;
                                }
                                else if(ori_conf == 1)
                                {
                                    bbox.alpha -= 3.1415926 / 2.0;
                                }

                                #endif

                                bbox.dim_h = sigmoid(*(info_3d_ptr + 3 * bin_num)) * 2.0 - 1.0 + dim_ref_[3 * cid];
                                bbox.dim_w = sigmoid(*(info_3d_ptr + 3 * bin_num + 1)) * 2.0 - 1.0 + dim_ref_[3 * cid + 1];
                                bbox.dim_l = sigmoid(*(info_3d_ptr + 3 * bin_num + 2)) * 2.0 - 1.0 + dim_ref_[3 * cid + 2];

                                //std::cout<< "bbox.cid : " << bbox.cid << std::endl;
                                bboxes->push_back(bbox);
                            }
                        }
                        ptr += 5 + num_class_ + num_3d_info_;
                    }
                }
            }
        }

        std::vector<Boundingbox> PostProcess(std::vector<float *> origin_output, float postThres, float nmsThres)
        {
            std::vector<Anchor> Anchors = InitAnchors();
            std::vector<Boundingbox> bboxes;
            #if ((Y3D_TYPE == 0) || (Y3D_TYPE == 2))
            std::vector<int> Strides = std::vector<int> {32, 16, 8};
            #elif (Y3D_TYPE == 1)
            std::vector<int> Strides = std::vector<int> {8, 16, 32};
            #endif
            for (int scale_idx=0; scale_idx<3; ++scale_idx) {
                const int stride = Strides[scale_idx];
                const int width = (input_width_ + stride - 1) / stride;
                const int height = (input_height_ + stride - 1) / stride;
                std::cout << "width : " << width << " " << "height : " << height << std::endl;
                float * cur_output_tensor = origin_output[scale_idx];
                PostProcessParall(height, width, scale_idx, postThres, cur_output_tensor, Strides, Anchors, &bboxes);
            }
            std::cout << " 3d bboxes size: " << bboxes.size()<< std::endl;
            Nmscpu(bboxes, nmsThres);
            return bboxes;
        }

        cv::Mat PreprocessImg(const cv::Mat& img)
        {
            int w, h, x, y;
            float r_w = input_width_ / (img.cols*1.0); // 640/1280 = 0.5
            float r_h = input_height_ / (img.rows*1.0); // 640/720 = 0.889
            if (r_h > r_w) {
                w = input_width_; // 640
                h = r_w * img.rows; // 720 * 0.5 = 360
                x = 0;
                y = (input_height_ - h) / 2; // (640-360)/2 = 90
            } else {
                w = r_h * img.cols;
                h = input_height_;
                x = (input_width_ - w) / 2;
                y = 0;
            }
            cv::Mat re(h, w, CV_8UC3); // 640*360*3
            cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
            //cv::Mat out(input_height_, input_width_, CV_8UC3, cv::Scalar(128, 128, 128));
            cv::Mat out(input_height_, input_width_, CV_8UC3, cv::Scalar(94, 95, 90));
            re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
            return out;
        }
//#if (Y3D_TYPE == 0)
#if ( (Y3D_TYPE == 0) || (Y3D_TYPE == 2) )
        ObjectType cvtToJmcobj(int cid)
        {
            #if 0
            enum class ObjectType {
            UNKNOWN = 0,
            UNKNOWN_MOVABLE = 1,
            UNKNOWN_UNMOVABLE = 2,
            PEDESTRIAN = 3,
            BICYCLE = 4,
            VEHICLE = 5,
            MAX_OBJECT_TYPE = 6,
            };
            #endif
           //names = ['Car', 'Van', 'Truck', 'Pedestrian','Person_sitting', 'Cyclist', 'Tram', 'Misc']
            ObjectType type = ObjectType::MAX_OBJECT_TYPE;
            if(cid == 3 || cid == 4)
            {
                type = ObjectType::PEDESTRIAN;
            }
            else if(cid == 5)
            {
                type = ObjectType::BICYCLE;
            }
            else if(cid == 0)
            {
                //CAR
                type = ObjectType::CAR;
            }
            else if(cid == 1)
            {
                //VAN/BUS
                type = ObjectType::BUS;
            }
            else if(cid == 2)
            {
                //TRUCK
                type = ObjectType::TRUCK;
            }
            else if(cid == 7)
            {
                type = ObjectType::UNKNOWN_UNMOVABLE;
            }
            else if(cid == 6)
            {
                type = ObjectType::UNKNOWN_MOVABLE;
            }
            /*
            else
            {
            type = ObjectType::UNKNOWN;
            }
            */
            return type;
        }

#elif (Y3D_TYPE == 1)
        ObjectType cvtToJmcobj(int cid)
        {
            #if 0
            enum class ObjectType {
            UNKNOWN = 0,
            UNKNOWN_MOVABLE = 1,
            UNKNOWN_UNMOVABLE = 2,
            PEDESTRIAN = 3,
            BICYCLE = 4,
            VEHICLE = 5,
            MAX_OBJECT_TYPE = 6,
            };
            #endif
            // ['Pedestrian', 'Car', 'Cyclist', 'Van', 'Truck',  'Person_sitting','Tram', 'Misc']
            ObjectType type = ObjectType::MAX_OBJECT_TYPE;
            if(cid == 0 || cid == 5)
            {
                type = ObjectType::PEDESTRIAN;
            }
            else if(cid == 2)
            {
                type = ObjectType::BICYCLE;
            }
            else if(cid == 1)
            {
                //CAR
                type = ObjectType::CAR;
            }
            else if(cid == 3)
            {
                //VAN/BUS
                type = ObjectType::BUS;
            }
            else if(cid == 4)
            {
                //TRUCK
                type = ObjectType::TRUCK;
            }
            else if(cid == 7)
            {
                type = ObjectType::UNKNOWN_UNMOVABLE;
            }
            else if(cid == 6)
            {
                type = ObjectType::UNKNOWN_MOVABLE;
            }
            /*
            else
            {
            type = ObjectType::UNKNOWN;
            }
            */
            return type;
        }
#endif

    };

}  // namespace perception
}  // namespace jmc_auto
#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_3D_DETECTOR_H_
