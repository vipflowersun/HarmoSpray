#pragma once
#include "AIS_Shape.hxx"
#include "AIS_InteractiveContext.hxx"
#include "AIS_Trihedron.hxx"
#include "Eigen/Dense"

class OccExtension
{
public:
    static bool areShapesIntersecting_OBB(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2);
    static bool areShapesIntersecting_Boolean(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2);
    // flag: 0 都按OBB  1 shape1按Mesh shape2按OBB 2 都按Mesh
    static bool areShapesIntersecting_Mesh(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2, int flag);
    static void drawOBB(const Eigen::Vector3d &min, const Eigen::Vector3d &max, double angle, const Handle(AIS_InteractiveContext) & context);
    static Handle(AIS_Trihedron) drawCoordinate(const gp_Ax2 &coordinateSystem, double size = 100);
    static std::list<Handle(AIS_Shape)> drawFloor(double gridSize, double gridStep);
};