#ifndef CNOID_AIST_COLLISION_DETECTOR_COLDET_MODEL_H
#define CNOID_AIST_COLLISION_DETECTOR_COLDET_MODEL_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <optional>
#include <string>
#include <vector>
#include "exportdecl.h"

namespace IceMaths {
class Matrix4x4;
}

namespace cnoid {

class ColdetModelInternalModel;

class CNOID_EXPORT ColdetModel : public Referenced
{
public:
    /**
       Primitive shape types.
       The order of the types follows that of SgMesh::PrimitiveType.
       The parameters of each primitive type set by setPrimitiveParam are as follows:
       - SP_BOX: half extents along the local x, y, z axes (3 parameters)
       - SP_SPHERE: radius
       - SP_CYLINDER: radius and height. The axis is along the local y-axis.
       - SP_CONE: radius of the base circle and height. The axis is along the
         local y-axis and the apex is at the +y end.
       - SP_CAPSULE: radius and the height of the cylindrical part.
         The axis is along the local y-axis.
    */
    enum PrimitiveType { SP_MESH, SP_BOX, SP_SPHERE, SP_CYLINDER, SP_CONE, SP_CAPSULE };

    /**
     * @brief constructor
     */
    ColdetModel();

    /**
     * @brief copy constructor
     *
     * Shape information stored in dataSet is shared with org
     */
    ColdetModel(const ColdetModel& org);

    /**
     * @brief destructor
     */
    virtual ~ColdetModel();

    virtual ColdetModel* clone() const;

    void cloneInternalModel();

    /**
     * @brief set name of this model
     * @param name name of this model 
     */
    void setName(const std::string& name) { name_ = name; }

    /**
     * @brief get name of this model
     * @return name name of this model 
     */
    const std::string& name() const { return name_; }

    /**
     * @brief set the number of vertices
     * @param n the number of vertices
     */
    void setNumVertices(int n);

    /**
     * @brief get the number of vertices
     * @return the number of vertices
     */
    int getNumVertices() const;

    /**
     * @brief set the number of triangles
     * @param n the number of triangles
     */
    void setNumTriangles(int n);

    int getNumTriangles() const;

    /**
     * @brief add a vertex
     * @param index index of the vertex
     * @param x x position of the vertex
     * @param y y position of the vertex
     * @param z z position of the vertex
     */
    void setVertex(int index, float x, float y, float z);

    /**
       add a vertex to the end of the vector
    */
    void addVertex(float x, float y, float z);

    /**
     * @brief get a vertex
     * @param index index of the vertex
     * @param out_x x position of the vertex
     * @param out_y y position of the vertex
     * @param out_z z position of the vertex
     */
    void getVertex(int index, float& out_x, float& out_y, float& out_z) const;

    /**
     * @brief add a triangle
     * @param index index of the triangle
     * @param v1 index of the first vertex
     * @param v2 index of the second vertex
     * @param v3 index of the third vertex
     */
    void setTriangle(int index, int v1, int v2, int v3);

    /**
       add a triangle to the end of the vector
    */
    void addTriangle(int v1, int v2, int v3);

    void getTriangle(int index, int& out_v1, int& out_v2, int& out_v3) const;

    /**
     * @brief build tree of bounding boxes to accelerate collision check
     *
     * This method must be called before doing collision check
     */
    void build();

    /**
     * @brief check if build() is already called or not
     * @return true if build() is already called, false otherwise
     */
    bool isValid() const { return isValid_; }

#ifdef CNOID_BACKWARD_COMPATIBILITY
    /**
     * @brief set position and orientation of this model
     * @param R new orientation 
     * @param p new position
     */
    void setPosition(const Matrix3& R, const Vector3& p);
#endif
    void setPosition(const Isometry3& T);
        
    /**
     * @brief set position and orientation of this model
     * @param R new orientation (row-major, length = 9)
     * @param p new position (length = 3)
     */
    void setPosition(const double* R, const double* p);

    /**
     * @brief set primitive type
     * @param ptype primitive type
     */
    void setPrimitiveType(PrimitiveType ptype);

    /**
     * @brief get primitive type
     * @return primitive type
     */
    PrimitiveType getPrimitiveType() const;

    /**
     * @brief set the number of parameters of primitive
     * @param nparam the number of parameters of primitive
     */
    void setNumPrimitiveParams(unsigned int nparam);

    /**
     * @brief set a parameter of primitive
     * @param index index of the parameter
     * @param value value of the parameter
     * @return true if the parameter is set successfully, false otherwise
     */
    bool setPrimitiveParam(unsigned int index, float value);

    /**
     * @brief get a parameter of primitive
     * @param index index of the parameter
     * @param value value of the parameter
     * @return true if the parameter is gotten successfully, false otherwise
     */
    bool getPrimitiveParam(unsigned int index, float &value) const;

    /**
     * @brief set position and orientation of primitive
     * @param R orientation relative to link (length = 9)
     * @param p position relative to link (length = 3)
     */
    void setPrimitivePosition(const double* R, const double* p);

    /**
       @brief set position and orientation of primitive relative to the model frame
    */
    void setPrimitiveLocalPosition(const Isometry3& T);
        
    /**
     * @brief compute distance between a point and this mesh along ray
     * @param point a point
     * @param dir direction of ray
     * @return distance if ray collides with this mesh
     */
    std::optional<double> computeDistanceWithRay(const double *point, const double *dir);

    /**
     * @brief check collision between this triangle mesh and a point cloud
     * @param i_cloud points
     * @param i_radius radius of spheres assigned to the points
     * @return true if colliding, false otherwise
     */
    bool checkCollisionWithPointCloud(const std::vector<Vector3> &i_cloud,
                                      double i_radius);

    void getBoundingBoxData(const int depth, std::vector<Vector3>& out_boxes);

    /**
       @brief get the axis-aligned bounding box of the whole model in the model-local frame
       @return true if the model has a valid bounding volume tree
    */
    bool getLocalBoundingBox(Vector3& out_center, Vector3& out_halfExtents) const;
        
    int getAABBTreeDepth();
    int getAABBmaxNum();
    int numofBBtoDepth(int minNumofBB);

private:
    void initialize();

    ColdetModelInternalModel* internalModel;
    IceMaths::Matrix4x4* transform;
    IceMaths::Matrix4x4* pTransform; ///< transform of primitive

    /**
       The following double-precision transforms hold the same information as
       the above single-precision transforms and are used by the analytic
       primitive shape collision detection.
    */
    Isometry3 position_;
    Isometry3 primitiveLocalPosition_;

    std::string name_;
    bool isValid_;

    friend class ColdetModelPair;
};

typedef ref_ptr<ColdetModel> ColdetModelPtr;

}

#endif
