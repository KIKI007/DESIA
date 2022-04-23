//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_ASSEMBLY_H
#define TI_STABLE_ASSEMBLY_H
#include "Part.h"

namespace voxel{
    class Assembly {
    public:

        typedef std::vector<Eigen::Vector3d> vecVector3d;

    public:
        Assembly();

        Assembly(int num)
        {
            set_part(num);
        }

        ~Assembly();

    public:

        void set_part(int num);

        //nrm : idA -> idB
        void add_contact(int idA, int idB, std::vector<Eigen::Vector3d> &nrms);

        //
        void add_fixed_part(int iA){ assert(0 <= iA && iA < parts_fixed_.size()); parts_fixed_[iA] = true;};
    public:

        std::vector<std::shared_ptr<Part>> partLists_;

        std::vector<bool> parts_fixed_;
    };

}


#endif //TI_STABLE_ASSEMBLY_H
