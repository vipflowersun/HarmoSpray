#include "OccExtension.h"

#include "BRepBuilderAPI_Transform.hxx"
#include "Bnd_OBB.hxx"
#include "BRepBndLib.hxx"
#include "BRepAlgoAPI_Section.hxx"
#include "TopExp_Explorer.hxx"
#include "Poly_Triangulation.hxx"
#include "TopoDS.hxx"
#include "BRep_Tool.hxx"
#include "BRepMesh_IncrementalMesh.hxx"
#include "AIS_Line.hxx"
#include "Geom_CartesianPoint.hxx"
#include "Geom_Axis2Placement.hxx"
#include "TopoDS_Edge.hxx"
#include "TopoDS_Face.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"

bool OccExtension::areShapesIntersecting_OBB(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2)
{
    TopoDS_Shape shape1TopoDS = shape1->Shape();
    TopoDS_Shape shape2TopoDS = shape2->Shape();

    BRepBuilderAPI_Transform transformer1(shape1TopoDS, shape1->LocalTransformation());
    shape1TopoDS = transformer1.Shape();
    BRepBuilderAPI_Transform transformer2(shape2TopoDS, shape2->LocalTransformation());
    shape2TopoDS = transformer2.Shape();

    Bnd_OBB bbox1;
    BRepBndLib::AddOBB(shape1TopoDS, bbox1);

    Bnd_OBB bbox2;
    BRepBndLib::AddOBB(shape2TopoDS, bbox2);

    return !bbox1.IsOut(bbox2);
}

bool OccExtension::areShapesIntersecting_Boolean(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2)
{
    // 获取 TopoDS_Shape 对象
    TopoDS_Shape shape1TopoDS = shape1->Shape();
    TopoDS_Shape shape2TopoDS = shape2->Shape();

    BRepBuilderAPI_Transform transformer1(shape1TopoDS, shape1->LocalTransformation());
    shape1TopoDS = transformer1.Shape();
    BRepBuilderAPI_Transform transformer2(shape2TopoDS, shape2->LocalTransformation());
    shape2TopoDS = transformer2.Shape();

    // 使用 BRepAlgoAPI_Section 进行布尔操作
    BRepAlgoAPI_Section section(shape1TopoDS, shape2TopoDS);

    // 执行布尔操作
    section.ComputePCurveOn1(Standard_True);
    section.Approximation(Standard_True);
    section.Build();

    // 检查结果
    if (!section.IsDone())
    {
        return false;
    }

    TopoDS_Shape result = section.Shape();
    if (result.IsNull())
    {
        return false;
    }

    // 使用 TopExp_Explorer 检查结果中是否有边（交线）
    TopExp_Explorer explorer(result, TopAbs_EDGE);
    return explorer.More();
}

static bool meshOBBIntersect(const Handle(Poly_Triangulation) & mesh,
                             const Bnd_OBB &OBB,
                             const gp_Trsf &meshMat)
{
    const TColgp_Array1OfPnt &nodes = mesh->Nodes();
    const Poly_Array1OfTriangle &triangles = mesh->Triangles();

    for (int i = triangles.Lower(); i <= triangles.Upper(); ++i)
    {
        Poly_Triangle triangle = triangles(i);
        Standard_Integer n1, n2, n3;
        triangle.Get(n1, n2, n3);

        Bnd_OBB triangleOBB;
        TColgp_Array1OfPnt pts(1, 3);
        pts.SetValue(1, nodes[n1].Transformed(meshMat));
        pts.SetValue(2, nodes[n2].Transformed(meshMat));
        pts.SetValue(3, nodes[n3].Transformed(meshMat));

        triangleOBB.ReBuild(pts);

        if (!triangleOBB.IsOut(OBB))
            return true;
    }
    return false;
}

static bool meshMeshIntersect(const Handle(Poly_Triangulation) & mesh1,
                              const Handle(Poly_Triangulation) & mesh2,
                              const gp_Trsf &mat1,
                              const gp_Trsf &mat2)
{
    const TColgp_Array1OfPnt &nodes1 = mesh1->Nodes();
    const Poly_Array1OfTriangle &triangles1 = mesh1->Triangles();

    const TColgp_Array1OfPnt &nodes2 = mesh2->Nodes();
    const Poly_Array1OfTriangle &triangles2 = mesh2->Triangles();

    for (int i = triangles1.Lower(); i <= triangles1.Upper(); ++i)
    {
        Poly_Triangle triangle1 = triangles1(i);
        Standard_Integer n1, n2, n3;
        triangle1.Get(n1, n2, n3);

        Bnd_OBB triangleOBB1;
        TColgp_Array1OfPnt pts1(1, 3);
        pts1.SetValue(1, nodes1[n1].Transformed(mat1));
        pts1.SetValue(2, nodes1[n2].Transformed(mat1));
        pts1.SetValue(3, nodes1[n3].Transformed(mat1));
        triangleOBB1.ReBuild(pts1);

        for (int j = triangles2.Lower(); j <= triangles2.Upper(); ++j)
        {
            Poly_Triangle triangle2 = triangles2(j);
            Standard_Integer n1_, n2_, n3_;
            triangle2.Get(n1_, n2_, n3_);

            Bnd_OBB triangleOBB2;
            TColgp_Array1OfPnt pts2(1, 3);
            pts2.SetValue(1, nodes2[n1_].Transformed(mat2));
            pts2.SetValue(2, nodes2[n2_].Transformed(mat2));
            pts2.SetValue(3, nodes2[n3_].Transformed(mat2));
            triangleOBB2.ReBuild(pts2);

            if (!triangleOBB1.IsOut(triangleOBB2))
                return true;
        }
    }
    return false;
}

bool OccExtension::areShapesIntersecting_Mesh(const Handle(AIS_Shape) & shape1, const Handle(AIS_Shape) & shape2, int flag)
{
    const double deflection = 10;

    if (flag != 1 && flag != 2)
        return OccExtension::areShapesIntersecting_OBB(shape1, shape2);

    // 计算OBB
    TopoDS_Shape shape1TopoDS = shape2->Shape();
    BRepBuilderAPI_Transform transformer1(shape1TopoDS, shape2->LocalTransformation());
    shape1TopoDS = transformer1.Shape();
    Bnd_OBB bbox1;
    BRepBndLib::AddOBB(shape1TopoDS, bbox1);
    TopoDS_Shape shape2TopoDS = shape2->Shape();
    BRepBuilderAPI_Transform transformer2(shape2TopoDS, shape2->LocalTransformation());
    shape2TopoDS = transformer2.Shape();
    Bnd_OBB bbox2;
    BRepBndLib::AddOBB(shape2TopoDS, bbox2);
    // OBB是否相交
    if (bbox1.IsOut(bbox2))
        return false;

    if (flag == 1)
    {
        // 计算Mesh
        std::vector<TopoDS_Face> topoFace1;
        TopoDS_Shape shape = shape1->Shape();
        gp_Trsf matShape1 = shape1->LocalTransformation();
        for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        {
            const TopoDS_Face &face = TopoDS::Face(explorer.Current());
            topoFace1.push_back(face);
        }
        // 计算相交
        for (const auto &face : topoFace1)
        {
            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
            if (triangulation.IsNull())
            {
                BRepMesh_IncrementalMesh mesher(face, deflection);
                triangulation = BRep_Tool::Triangulation(face, loc);
            }
            if (meshOBBIntersect(triangulation, bbox2, matShape1))
                return true;
        }
    }
    else if (flag == 2)
    {
        // 计算Mesh
        std::vector<Handle(Poly_Triangulation)> triangle1;
        gp_Trsf matShape1 = shape1->LocalTransformation();
        for (TopExp_Explorer explorer(shape1->Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
        {
            const TopoDS_Face &face = TopoDS::Face(explorer.Current());
            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
            if (triangulation.IsNull())
            {
                BRepMesh_IncrementalMesh mesher(face, deflection);
                triangulation = BRep_Tool::Triangulation(face, loc);
            }
            if (!triangulation.IsNull())
            {
                triangle1.push_back(triangulation);
            }
        }
        std::vector<Handle(Poly_Triangulation)> triangle2;
        gp_Trsf matShape2 = shape2->LocalTransformation();
        for (TopExp_Explorer explorer(shape2->Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
        {
            const TopoDS_Face &face = TopoDS::Face(explorer.Current());
            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
            if (triangulation.IsNull())
            {
                BRepMesh_IncrementalMesh mesher(face, deflection);
                triangulation = BRep_Tool::Triangulation(face, loc);
            }
            if (!triangulation.IsNull())
            {
                triangle2.push_back(triangulation);
            }
        }
        // 计算相交
        for (const auto &trian1 : triangle1)
        {
            for (const auto &trian2 : triangle2)
            {
                if (meshMeshIntersect(trian1, trian2, matShape1, matShape2))
                    return true;
            }
        }
    }

    return false;
}

void OccExtension::drawOBB(const Eigen::Vector3d &min, const Eigen::Vector3d &max, double angle, const Handle(AIS_InteractiveContext) & context)
{
    Handle(AIS_Line) lines[5] = {0};
    lines[0] = new AIS_Line(new Geom_CartesianPoint(min(0), min(1), 0),
                            new Geom_CartesianPoint(max(0), min(1), 0));
    lines[1] = new AIS_Line(new Geom_CartesianPoint(max(0), min(1), 0),
                            new Geom_CartesianPoint(max(0), max(1), 0));
    lines[2] = new AIS_Line(new Geom_CartesianPoint(max(0), max(1), 0),
                            new Geom_CartesianPoint(min(0), max(1), 0));
    lines[3] = new AIS_Line(new Geom_CartesianPoint(min(0), max(1), 0),
                            new Geom_CartesianPoint(min(0), min(1), 0));
    lines[4] = new AIS_Line(new Geom_CartesianPoint(min(0), min(1), 0),
                            new Geom_CartesianPoint(max(0), max(1), 0));
    gp_Trsf mat;
    mat.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), angle);
    for (const auto &line : lines)
    {
        line->SetColor(Quantity_NOC_GREEN);
        line->SetLocalTransformation(mat);
        context->Display(line, Standard_False);
        context->Deactivate(line);
    }
}

Handle(AIS_Trihedron) OccExtension::drawCoordinate(const gp_Ax2 &coordinateSystem, double size /*= 100*/)
{
    // 创建一个三维坐标系的展示对象
    Handle(Geom_Axis2Placement) axis2Placement = new Geom_Axis2Placement(coordinateSystem);
    Handle(AIS_Trihedron) trihedron = new AIS_Trihedron(axis2Placement);
    trihedron->SetSize(size);

    // 设置颜色
    trihedron->SetTextColor(Quantity_NOC_BLACK);
    trihedron->SetOriginColor(Quantity_NOC_WHITE);
    trihedron->SetXAxisColor(Quantity_NOC_RED);
    trihedron->SetDatumPartColor(Prs3d_DP_XArrow, Quantity_NOC_RED);
    trihedron->SetYAxisColor(Quantity_NOC_GREEN);
    trihedron->SetDatumPartColor(Prs3d_DP_YArrow, Quantity_NOC_GREEN);
    trihedron->SetAxisColor(Quantity_NOC_BLUE1);
    trihedron->SetDatumPartColor(Prs3d_DP_ZArrow, Quantity_NOC_BLUE1);
    trihedron->SetWidth(20);

    return trihedron;
}

std::list<Handle(AIS_Shape)> OccExtension::drawFloor(double gridSize, double gridStep)
{
    std::list<Handle(AIS_Shape)> floor;

    if (gridSize <= 0 || gridStep <= 0)
        return floor;

    // 网格参数
    int numLines = static_cast<int>(gridSize / gridStep);

    // X方向线
    for (int i = -numLines / 2; i <= numLines / 2; ++i)
    {
        double x = i * gridStep;
        gp_Pnt p1(x, -gridSize / 2, 0);
        gp_Pnt p2(x, gridSize / 2, 0);
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
        Handle(AIS_Shape) aisLine = new AIS_Shape(edge);
        aisLine->SetColor(Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB)); // 灰色
        aisLine->SetWidth(x == 0 ? 2.0 : 1.0);
        floor.push_back(aisLine);
    }

    // Y方向线
    for (int i = -numLines / 2; i <= numLines / 2; ++i)
    {
        double y = i * gridStep;
        gp_Pnt p1(-gridSize / 2, y, 0);
        gp_Pnt p2(gridSize / 2, y, 0);
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
        Handle(AIS_Shape) aisLine = new AIS_Shape(edge);
        aisLine->SetColor(Quantity_Color(0.5, 0.5, 0.5, Quantity_TOC_RGB));
        aisLine->SetWidth(y == 0 ? 2.0 : 1.0);
        floor.push_back(aisLine);
    }

    gp_Pln plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    TopoDS_Face face = BRepBuilderAPI_MakeFace(plane, -gridSize / 2, gridSize / 2, -gridSize / 2, gridSize / 2);
    Handle(AIS_Shape) aisFace = new AIS_Shape(face);
    aisFace->SetColor(Quantity_Color(0.7, 0.7, 0.7, Quantity_TOC_RGB));
    aisFace->SetTransparency(0.7);

    floor.push_back(aisFace);

    return floor;
}
