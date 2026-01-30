/*输出信息窗口类*/
#pragma once

#include <QTextBrowser>
#include <QVector>
#include <QFile>
#include <QTextStream>

#define BLACKFONT "<font color=\"#000000\">"
#define REDFONT "<font color=\"#FF0000\">"
#define GREENFONT "<font color=\"#00AA00\">"
#define YELLOWFONT "<font color=\"#AAAA00\">"

class OutputWidget : public QTextBrowser
{
	typedef struct _ColorString
	{
		_ColorString() = default;
		_ColorString(const QString _str, QString _Color)
		{

			str = _str;
			Color = _Color;
		}
		QString str;
		QString Color;
	} ColorString;
	Q_OBJECT
public:
	OutputWidget(QWidget *parent = Q_NULLPTR);
	~OutputWidget();

public:
	void setLogfilePath(QString LogFilePath);
	void log(const QString& str);
	void warning(const QString& str);
	void error(const QString& str);

private:
	static QVector<ColorString> InfoStack; // 窗口未构造前的Info堆栈
	QString LogFilePath;
	QFile LogFile;
	QTextStream out;
	bool IsOutPutLogFile; // 向log日志输出开关
};