//
// Created by *** on 18.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_COLORCODING_H
#define UNDERSTAND_INTERLOCK_COLORCODING_H

#include <Eigen/Dense>
#include <vector>
#include "igl/jet.h"

namespace voxel {

    class ColorCoding{
    public:
        typedef Eigen::Vector4d Color;
        typedef std::vector<Eigen::Vector4d> veccolor;
    public:
        ColorCoding(const veccolor& tab)
        {
            part_num_ = tab.size();

            for (int id = 0; id < part_num_; id++) {
                colorCode.push_back(tab[id]);
            }
        }

    public:
        void request(int num) {

            for (int id = part_num_; id < num; id++) {
                Eigen::RowVector3d c, oc;
                while (true) {
                    c = Eigen::RowVector3d(rand() % 255 / 255.0,
                                           rand() % 255 / 255.0,
                                           rand() % 255 / 255.0);
                    int jd;
                    for (jd = 0; jd < 10; jd++) {
                        oc = Eigen::RowVector3d((colorCode[jd]).x(), (colorCode[jd]).y(), (colorCode[jd]).x());
                        if ((c - oc).norm() < 0.4) break;
                    }
                    if (jd == 10) {
                        break;
                    }
                }
                Color pC(c[0] * 255, c[1] * 255, c[2] * 255, 255);
                colorCode.push_back(pC);
            }
            part_num_ = num;
        }

        Eigen::RowVector3d get(int id) {
            if (id < 0 || id >= part_num_) return Eigen::RowVector3d(0, 0, 0);
            else return Eigen::RowVector3d(colorCode[id].x(), colorCode[id].y(), colorCode[id].z());
        }

    public:

        void jet(Eigen::VectorXd &value, double min, double max, Eigen::MatrixXd &C) {
            igl::jet(value, min, max, C);
        }

    public:

        int part_num_;

        veccolor colorCode;
    };
}


#endif //UNDERSTAND_INTERLOCK_COLORCODING_H
