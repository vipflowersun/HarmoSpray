#include "OccView.h"
#include "occt_window.h"

#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <AIS_Axis.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <STEPControl_Reader.hxx>
#include <Geom_Axis1Placement.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <StlAPI_Reader.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_OBB.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <QApplication>
#include <QScreen>
#include <QMouseEvent>
#include <QDebug>

OccView::OccView(QWidget *parent) noexcept
    : QWidget{parent}
{
    // 直接绘制在屏幕上
    this->setAttribute(Qt::WA_PaintOnScreen);
    // 创建连接显示设备
    Handle(Aspect_DisplayConnection) m_Aspect_DisplayConnect = new Aspect_DisplayConnection();
    // 创建3D接口定义图形驱动
    Handle(OpenGl_GraphicDriver) driver = new OpenGl_GraphicDriver(m_Aspect_DisplayConnect);
    // 创建3D查看器对象，并指定图形驱动
    _viewer = new V3d_Viewer(driver);
    // 创建交互上下文对象，关联到3D查看器
    _context = new AIS_InteractiveContext(_viewer);
    // 创建视图，并关联到3D查看器
    _view = _viewer->CreateView();
    // 获取窗口句柄并创建OCCT_Window
    OCCT_Window* wind = new OCCT_Window(this);
    // 设置视图窗口
    _view->SetWindow(wind);
    if (!wind->IsMapped()) wind->Map();

    // Set up lights etc
    _viewer->SetDefaultLights();
    _viewer->SetLightOn();

    Quantity_Color aWhite(Quantity_NOC_WHITE);
    Quantity_Color aGray(Quantity_NOC_GRAY);
    _view->SetBgGradientColors(aWhite, aGray, Aspect_GFM_CORNER1);
    _view->MustBeResized();
    _view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_BLACK, 0.08, V3d_ZBUFFER);
    Graphic3d_RenderingParams& aParams = _view->ChangeRenderingParams();
    aParams.IsAntialiasingEnabled = TRUE;
    aParams.NbMsaaSamples = GL_MAX_SAMPLES;

    _context->SetDisplayMode(AIS_Shaded, Standard_True);

    //设置高亮颜色
    Handle(Prs3d_Drawer) HighlightStyle = new Prs3d_Drawer();
    Quantity_Color HightlightColor(1, 0.549, 0, Quantity_TOC_RGB);
    HighlightStyle->SetColor(HightlightColor);
    Handle(Prs3d_LineAspect) lineAspect = new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, 4);
    HighlightStyle->SetLineAspect(lineAspect);
    HighlightStyle->SetDisplayMode(AIS_WireFrame);
    _context->SetHighlightStyle(Prs3d_TypeOfHighlight_Dynamic, HighlightStyle);
    _context->SetHighlightStyle(Prs3d_TypeOfHighlight_LocalDynamic, HighlightStyle);

    Handle(Prs3d_Drawer) LocalSelectStyle = new Prs3d_Drawer();
    Quantity_Color LocalSelectColor(0, 0, 1, Quantity_TOC_RGB);
    LocalSelectStyle->SetColor(LocalSelectColor);
    LocalSelectStyle->SetTransparency(0.3f);
    LocalSelectStyle->SetDisplayMode(AIS_Shaded);
    _context->SetHighlightStyle(Prs3d_TypeOfHighlight_LocalSelected, LocalSelectStyle);

    Handle(Prs3d_Drawer) SelectStyle = new Prs3d_Drawer();
    Quantity_Color SelectColor(0, 0, 1, Quantity_TOC_RGB);
    SelectStyle->SetColor(SelectColor);
    SelectStyle->SetDisplayMode(AIS_Shaded);
    _context->SetHighlightStyle(Prs3d_TypeOfHighlight_Selected, SelectStyle);

    _pixelRatio = qApp->primaryScreen()->devicePixelRatio();
    setMouseTracking(true);
}

const Handle(AIS_InteractiveContext)& OccView::getContext() const
{
    return _context;
}

const Handle(V3d_View)& OccView::getView() const
{
    return _view;
}

Handle(AIS_Shape_WithFrame) OccView::displayTopo(const TopoDS_Shape& topo, bool withFrame /*= false*/)
{
    Handle(AIS_Shape_WithFrame) anAisBox = new AIS_Shape_WithFrame(topo);
    anAisBox->SetDisplayMode(AIS_Shaded);
    anAisBox->SetWidth(4);
    _context->Display(anAisBox, Standard_False);

    //_context->Deactivate(anAisBox, TopAbs_COMPOUND);
    //_context->Deactivate(anAisBox, TopAbs_COMPSOLID);
    //_context->Deactivate(anAisBox, TopAbs_SOLID);
    //_context->Deactivate(anAisBox, TopAbs_SHELL);
    //_context->Activate(anAisBox, TopAbs_FACE);
    //_context->Deactivate(anAisBox, TopAbs_WIRE);
    //_context->Deactivate(anAisBox, TopAbs_EDGE);
    //_context->Deactivate(anAisBox, TopAbs_VERTEX);
    //_context->Deactivate(anAisBox, TopAbs_SHAPE);

    if (withFrame)
    {
        _context->Display(anAisBox->frameShape(), Standard_False);
        _context->Deactivate(anAisBox->frameShape());
    }

    return anAisBox;
}

void OccView::displayAIS(const Handle(AIS_Shape_WithFrame)& ais, bool withFrame)
{
    ais->SetDisplayMode(AIS_Shaded);
    ais->SetWidth(4);
    _context->Display(ais, Standard_False);

    //_context->Deactivate(ais);

    //_context->Activate(ais, AIS_Shape::SelectionMode(TopAbs_FACE));

    if (withFrame)
    {
        _context->Display(ais->frameShape(), Standard_False);
        _context->Deactivate(ais->frameShape());
    }

    if (Handle(AIS_Shape_WithCoord) shapeCoord = Handle(AIS_Shape_WithCoord)::DownCast(ais))
    {
        _context->Display(shapeCoord->coordShape(), Standard_False);
        _context->Deactivate(shapeCoord->coordShape());
    }
}

TopoDS_Shape OccView::readStepFile(const QString& filename)
{
    STEPControl_Reader reader;
    if (reader.ReadFile(std::string(filename.toLocal8Bit()).c_str()) != IFSelect_RetDone)
    {
        return TopoDS_Shape();
    }

    // 读取模型
    reader.TransferRoots();
    return reader.OneShape();
}

TopoDS_Shape OccView::readStlFile(const QString filename)
{
    StlAPI_Reader aReader_Stl;
    TopoDS_Shape shape_Stl;
    aReader_Stl.Read(shape_Stl, filename.toLocal8Bit());

    return shape_Stl;
}

void OccView::displayCoord(const gp_Pnt& origin, const gp_Dir& xDir, const gp_Dir& yDir, const gp_Dir& zDir)
{
    Handle(AIS_Axis) xAxis = new AIS_Axis(new Geom_Axis1Placement(gp_Ax1(origin, xDir)));
    Handle(AIS_Axis) yAxis = new AIS_Axis(new Geom_Axis1Placement(gp_Ax1(origin, yDir)));
    Handle(AIS_Axis) zAxis = new AIS_Axis(new Geom_Axis1Placement(gp_Ax1(origin, zDir)));

    xAxis->SetColor(Quantity_NOC_RED);
    yAxis->SetColor(Quantity_NOC_GREEN);
    zAxis->SetColor(Quantity_NOC_BLUE1);

    _context->Display(xAxis, Standard_False);
    _context->Display(yAxis, Standard_False);
    _context->Display(zAxis, Standard_False);

    _context->Deactivate(xAxis);
    _context->Deactivate(yAxis);
    _context->Deactivate(zAxis);

    this->update();
}

std::vector<TopoDS_Shape> OccView::querySelection(std::vector<Handle(AIS_InteractiveObject)>& AISObj) const
{
    std::vector<TopoDS_Shape> shapeSequence;

    for (_context->InitSelected(); _context->MoreSelected(); _context->NextSelected())
    {
        Handle(AIS_InteractiveObject)  aisObj = _context->SelectedInteractive();
        AISObj.push_back(aisObj);
        TopoDS_Shape aShape = _context->SelectedShape();
        shapeSequence.push_back(aShape);
    }

    return shapeSequence;
}

void OccView::fitAll(void)
{
	_view->FitAll();
	_view->ZFitAll();
	_view->Redraw();
}

void OccView::updateView(void)
{
    _view->Redraw();
}

void OccView::reset(void)
{
	_view->Reset();
}

QPaintEngine* OccView::paintEngine() const
{
    return nullptr;
}

void OccView::paintEvent( QPaintEvent* /*theEvent*/ )
{
    _view->Redraw();
}

void OccView::resizeEvent( QResizeEvent* /*theEvent*/ )
{
    if( !_view.IsNull() )
    {
        _view->MustBeResized();
    }
}

void OccView::mousePressEvent(QMouseEvent* theEvent)
{
    if (!_context)
    {
        //theEvent->ignore();
        return QWidget::mousePressEvent(theEvent);
    }

    QPoint pos = theEvent->pos() * _pixelRatio;

    if (theEvent->button() == Qt::MiddleButton && theEvent->modifiers() != Qt::ControlModifier)
    {
        _view->StartRotation(pos.x(), pos.y());
    }

    //theEvent->ignore();
    return QWidget::mousePressEvent(theEvent);
}

void OccView::mouseReleaseEvent(QMouseEvent* theEvent)
{
    if (!_context)
    {
        //theEvent->ignore();
        return QWidget::mouseReleaseEvent(theEvent);
    }

    if (theEvent->button() == Qt::LeftButton)
    {
        if (theEvent->modifiers() == Qt::ControlModifier)
            _context->ShiftSelect(Standard_True);
        else
            _context->Select(Standard_True);

        emit selectionChanged();
    }

    //theEvent->ignore();
    return QWidget::mouseReleaseEvent(theEvent);
}

void OccView::mouseMoveEvent(QMouseEvent* theEvent)
{
    if (!_context)
    {
        //theEvent->ignore();
        return QWidget::mouseMoveEvent(theEvent);
    }

    QPoint pos = theEvent->pos() * _pixelRatio;

    _context->MoveTo(pos.x(), pos.y(), _view, Standard_True);

    if (theEvent->buttons() == Qt::MiddleButton)
    {
        if (theEvent->modifiers() == Qt::ControlModifier)
        {
            _view->Pan(pos.x() - _mousePosRec.x(), _mousePosRec.y() - pos.y());
        }
        else
        {
            _view->Rotation(pos.x(), pos.y());
        }
    }

    _mousePosRec = pos;

    //theEvent->ignore();
    return QWidget::mouseMoveEvent(theEvent);
}

void OccView::wheelEvent(QWheelEvent* theEvent)
{
    if (!_context)
    {
        //theEvent->ignore();
        return QWidget::wheelEvent(theEvent);
    }

    QPointF pos = theEvent->position() * _pixelRatio;

    _view->StartZoomAtPoint(pos.x(), pos.y());
    _view->ZoomAtPoint(0, 0, theEvent->angleDelta().y() * 0.1, 0); //执行缩放

    //theEvent->ignore();
    QWidget::wheelEvent(theEvent);
}


