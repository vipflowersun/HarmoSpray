#pragma once
#include "AIS_Shape.hxx"
#include "AIS_InteractiveContext.hxx"
#include "AIS_DisplayMode.hxx"
#include "AIS_Trihedron.hxx"
#include "Geom_Axis2Placement.hxx"

#include "Eigen\Dense"

#include <any>
#include <map>

class AIS_Shape_WithFrame : public AIS_Shape
{
public:
    inline AIS_Shape_WithFrame(const TopoDS_Shape &shape) : AIS_Shape(shape)
    {
        _frameShape = new AIS_Shape(shape);
        _frameShape->SetColor(Quantity_NOC_BLACK);
        _frameShape->SetDisplayMode(AIS_WireFrame);
    }
    inline virtual Handle(AIS_Shape_WithFrame) clone()
    {
        Handle(AIS_Shape_WithFrame) cloneShape = new AIS_Shape_WithFrame(this->Shape());
        cloneShape->SetLocalTransformation(this->LocalTransformation());
        Quantity_Color color;
        this->Color(color);
        cloneShape->SetColor(color);
        cloneShape->SetTransparency(this->Transparency());
        return cloneShape;
    }
    inline virtual void SetLocalTransformation(const gp_Trsf &theTrsf)
    {
        AIS_Shape::SetLocalTransformation(theTrsf);
        if (_frameShape)
            _frameShape->SetLocalTransformation(theTrsf);
    }
    inline virtual void SetTransparency(const Standard_Real aValue = 0.6)
    {
        AIS_Shape::SetTransparency(aValue);
        if (_frameShape)
            _frameShape->SetTransparency(aValue);
    }
    inline const Handle(AIS_Shape) & frameShape() const
    {
        return _frameShape;
    }
    inline std::string name() const
    {
        return _name;
    }
    inline void setName(const std::string &name)
    {
        _name = name;
    }
    inline void setUserData(const std::string &key, const std::any &value)
    {
        _userData[key] = value;
    }
    template <typename T>
    T getUserData(const std::string &key);

private:
    Handle(AIS_Shape) _frameShape = nullptr;
    std::string _name;
    std::map<std::string, std::any> _userData;
};

template <typename T>
T AIS_Shape_WithFrame::getUserData(const std::string &key)
{
    auto it = _userData.find(key);
    if (it == _userData.end())
        return {};
    return std::any_cast<T>(it->second);
}

class AIS_Shape_WithCoord : public AIS_Shape_WithFrame
{
public:
    AIS_Shape_WithCoord(const TopoDS_Shape &shape, const gp_Ax2 &coordinateSystem, double size = 100) : AIS_Shape_WithFrame(shape), _coordinateSystem(coordinateSystem)
    {
        _coordShape = this->drawCoordinate(coordinateSystem, size);
    }
    inline virtual Handle(AIS_Shape_WithFrame) clone() override
    {
        Handle(AIS_Shape_WithCoord) cloneShape = new AIS_Shape_WithCoord(this->Shape(), _coordinateSystem, _coordShape->Size());
        cloneShape->SetLocalTransformation(this->LocalTransformation());
        Quantity_Color color;
        this->Color(color);
        cloneShape->SetColor(color);
        cloneShape->SetTransparency(this->Transparency());
        return cloneShape;
    }
    inline virtual void SetLocalTransformation(const gp_Trsf &theTrsf) override
    {
        AIS_Shape_WithFrame::SetLocalTransformation(theTrsf);
        if (_coordShape)
            _coordShape->SetLocalTransformation(theTrsf);
    }
    inline virtual void SetTransparency(const Standard_Real aValue = 0.6) override
    {
        AIS_Shape_WithFrame::SetTransparency(aValue);
        if (_coordShape)
            _coordShape->SetTransparency(aValue);
    }
    void setCoordVisable(bool visable)
    {
        if (!_coordShape || !_coordShape->GetContext())
            return;
        if (visable)
            _coordShape->GetContext()->Display(_coordShape, Standard_False);
        else
            _coordShape->GetContext()->Erase(_coordShape, Standard_False);
    }
    void setCoordSize(double size)
    {
        if (!_coordShape)
            return;
        _coordShape->SetSize(size);
    }
    void setCoordLocate(const Eigen::Matrix4d &mat)
    {
        gp_Pnt origin(mat(0, 3), mat(1, 3), mat(2, 3));
        gp_Dir xDir(mat(0, 0), mat(1, 0), mat(2, 0));
        gp_Dir yDir(mat(0, 1), mat(1, 1), mat(2, 1));
        gp_Dir zDir(mat(0, 2), mat(1, 2), mat(2, 2));
        gp_Ax2 ax2(origin, zDir, xDir);
        int size = _coordShape->Size();
        auto localMat = _coordShape->LocalTransformation();
        double transparency = _coordShape->Transparency();
        auto contex = _coordShape->GetContext();
        if (contex)
            contex->Remove(_coordShape, Standard_False);
        _coordShape = this->drawCoordinate(ax2, size);
        _coordShape->SetLocalTransformation(localMat);
        _coordShape->SetTransparency(transparency);
        if (contex)
        {
            contex->Display(_coordShape, Standard_False);
            contex->Deactivate(_coordShape);
        }
    }
    Handle(AIS_Trihedron) coordShape() const
    {
        return _coordShape;
    }

private:
    Handle(AIS_Trihedron) drawCoordinate(const gp_Ax2 &coordinateSystem, double size /*= 100*/)
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

private:
    Handle(AIS_Trihedron) _coordShape = nullptr;
    gp_Ax2 _coordinateSystem;
};