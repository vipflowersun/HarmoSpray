#pragma once
#include <QWidget>

#include <any>

#include <V3d_Viewer.hxx>
#include <V3d_view.hxx>
#include <AIS_InteractiveContext.hxx>

#include "AIS_Shape_WithFrame.hpp"

class OccView : public QWidget
{
	Q_OBJECT
public:
	explicit OccView(QWidget *parent = nullptr) noexcept;

public:
	const Handle(AIS_InteractiveContext) & getContext() const;
	const Handle(V3d_View) & getView() const;
	Handle(AIS_Shape_WithFrame) displayTopo(const TopoDS_Shape &topo, bool withFrame = false);
	void displayAIS(const Handle(AIS_Shape_WithFrame) & ais, bool withFrame = false);
	TopoDS_Shape readStepFile(const QString &filename);
	TopoDS_Shape readStlFile(const QString filename);
	void displayCoord(const gp_Pnt &origin, const gp_Dir &xDir, const gp_Dir &yDir, const gp_Dir &zDir);
	std::vector<TopoDS_Shape> querySelection(std::vector<Handle(AIS_InteractiveObject)> &AISObj) const;
	template <class T>
	std::vector<Handle(T)> findAISShape(std::function<bool(Handle(T))> f = nullptr);
	template <class T>
	Handle(T) findAISShape(const std::string &name);
	template <class T>
	void removeAISShape(std::function<bool(Handle(T))> f = nullptr);
signals:
	void selectionChanged(void);
public slots:
	//! operations for the view.
	void fitAll(void);
	void updateView(void);
	void reset(void);

protected:
	virtual QPaintEngine *paintEngine() const override;
	virtual void paintEvent(QPaintEvent * /*theEvent*/) override;
	virtual void resizeEvent(QResizeEvent * /*theEvent*/) override;
	virtual void mousePressEvent(QMouseEvent *theEvent) override;
	virtual void mouseReleaseEvent(QMouseEvent *theEvent) override;
	virtual void mouseMoveEvent(QMouseEvent *theEvent) override;
	virtual void wheelEvent(QWheelEvent *theEvent) override;

private:
	Handle(V3d_Viewer) _viewer = nullptr;
	Handle(V3d_View) _view = nullptr;
	Handle(AIS_InteractiveContext) _context = nullptr;

	double _pixelRatio = 1; // 系统屏幕缩放比例
	QPoint _mousePosRec;
};

template <class T>
std::vector<Handle(T)> OccView::findAISShape(std::function<bool(Handle(T))> f /*= nullptr*/)
{
	std::vector<Handle(T)> Ret;
	// 遍历
	AIS_ListOfInteractive displayedObjects;
	_context->DisplayedObjects(displayedObjects);
	for (AIS_ListIteratorOfListOfInteractive it(displayedObjects); it.More(); it.Next())
	{
		Handle(AIS_InteractiveObject) currentObject = it.Value();
		if (Handle(T) DirEdge = Handle(T)::DownCast(currentObject))
		{
			if (!f || (f && f(DirEdge)))
				Ret.push_back(DirEdge);
		}
	}
	return Ret;
}

template <class T>
Handle(T) OccView::findAISShape(const std::string &name)
{
	// 遍历
	AIS_ListOfInteractive displayedObjects;
	_context->DisplayedObjects(displayedObjects);
	for (AIS_ListIteratorOfListOfInteractive it(displayedObjects); it.More(); it.Next())
	{
		Handle(AIS_InteractiveObject) currentObject = it.Value();
		Handle(T) TShape = Handle(T)::DownCast(currentObject);
		Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(currentObject);
		if (TShape && frameShape && frameShape->name() == name)
		{
			return TShape;
		}
	}

	return nullptr;
}

template <class T>
void OccView::removeAISShape(std::function<bool(Handle(T))> f /*= nullptr*/)
{
	// 遍历
	AIS_ListOfInteractive displayedObjects;
	_context->DisplayedObjects(displayedObjects);
	for (AIS_ListIteratorOfListOfInteractive it(displayedObjects); it.More(); it.Next())
	{
		Handle(AIS_InteractiveObject) currentObject = it.Value();
		if (Handle(T) shape = Handle(T)::DownCast(currentObject))
		{
			if (!f || (f && f(shape)))
				_context->Remove(shape, Standard_False);
			if (Handle(AIS_Shape_WithFrame) shapeWithFrame = Handle(AIS_Shape_WithFrame)::DownCast(currentObject))
			{
				_context->Remove(shapeWithFrame->frameShape(), Standard_False);
			}
			if (Handle(AIS_Shape_WithCoord) shapeCoord = Handle(AIS_Shape_WithCoord)::DownCast(currentObject))
			{
				_context->Remove(shapeCoord->coordShape(), Standard_False);
			}
		}
	}
}