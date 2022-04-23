//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
#define TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef Eigen::Vector3d EigenPoint;
using std::vector;

typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> EigenPointList;

class ContactPolygon{
public:
    EigenPointList points;

    EigenPoint center;
};

/*!
 * \brief: Stores which parts are contacted and the contact's geometry
 */
class ContactGraphEdge
{
public:
    vector<ContactPolygon> polygons;  //!< a list which polygons are simple and closed (better to be convex)

    vector<EigenPoint> normals; //!< contact normal from partIDA to partIDB

public:

    ContactGraphEdge(ContactPolygon &_polygon, EigenPoint &_normal)
    {
        polygons.push_back(_polygon);
        normals.push_back(_normal);
    }

    ContactGraphEdge(vector<ContactPolygon> &_polygons, vector<EigenPoint> &_normals)
    {
        polygons = _polygons;
        normals = _normals;
    }

    /*!
     * \return the contact normal starts from partID
     */
    bool getContactNormal(int partID, vector<EigenPoint> &_normals){
        if(partIDA != partID && partIDB != partID) return false;
        if(partIDA == partID){
            _normals = normals;
        }
        if(partIDB == partID){
            for(EigenPoint nrm : normals)
            {
                _normals.push_back(nrm * (-1.0));
            }
        }
        return true;
    }

public: //Automatic Generate

    int partIDA; //!< Static ID for start node of this edge

    int partIDB; //!< Static ID for end node of this edge
};



#endif //TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
