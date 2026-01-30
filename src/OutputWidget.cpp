#include "OutputWidget.h"
#include <QFile>
#include <QTextStream>
#include <QDateTime>

QVector<OutputWidget::ColorString> OutputWidget::InfoStack;

OutputWidget::OutputWidget(QWidget *parent)
	: QTextBrowser(parent), IsOutPutLogFile(false)
{
	LogFilePath = QDateTime::currentDateTime().toString("yyyyMMdd_hh-mm-ss") + "UserLog.txt";
	LogFile.setFileName(LogFilePath);
	if (IsOutPutLogFile == true)
	{
		LogFile.open(QIODevice::WriteOnly | QIODevice::Append);
		out.setDevice(&LogFile);
	}
	// 构造前的Info
	if (InfoStack.size() == 0)
		return;
	foreach (ColorString Info_i, InfoStack)
	{
		QString text;
		QString current_date_time = QDateTime::currentDateTime().toString("hh:mm:ss");
		QString current_date = QString("%1").arg(current_date_time);
		text = current_date;
		text += ":   ";
		text += Info_i.str;
		this->append(Info_i.Color + text + "</font> ");
		// 向log文件输出
		if (IsOutPutLogFile == false)
			continue;
		if (Info_i.Color == REDFONT)
			text = current_date + "[Warning]";
		else
			text = current_date;
		text += ":   ";
		text += Info_i.str;
		out << text;
		out << "\n";
	}
}

OutputWidget::~OutputWidget()
{
	LogFile.close();
}

void OutputWidget::setLogfilePath(QString LogFilePath)
{
	this->LogFilePath = LogFilePath;
}

void OutputWidget::log(const QString& str)
{
	QString text;
	QString current_date_time = QDateTime::currentDateTime().toString("(MM-dd/ hh:mm:ss)");
	text = BLACKFONT + current_date_time + "</font>";
	text += GREENFONT + str + "</font>";
	QMetaObject::invokeMethod(this, "append", Qt::QueuedConnection, Q_ARG(QString, text));
	// 向log文件输出
	if (IsOutPutLogFile == false)
		return;
	text = "[Log]" + QString("(") + current_date_time + QString(")");
	text += ":   ";
	text += str;
	out << text;
	out << "\n";
}

void OutputWidget::warning(const QString& str)
{
	QString text;
	QString current_date_time = QDateTime::currentDateTime().toString("(MM-dd/ hh:mm:ss)");
	text = BLACKFONT + current_date_time + "</font>";
	text += YELLOWFONT + str + "</font>";
	QMetaObject::invokeMethod(this, "append", Qt::QueuedConnection, Q_ARG(QString, text));
	// 向log文件输出
	if (IsOutPutLogFile == false)
		return;
	text = "[Warning]" + QString("(") + current_date_time + QString(")");
	text += ":   ";
	text += str;
	out << text;
	out << "\n";
}

void OutputWidget::error(const QString& str)
{
	QString text;
	QString current_date_time = QDateTime::currentDateTime().toString("(MM-dd/ hh:mm:ss)");
	text = BLACKFONT + current_date_time + "</font>";
	text += REDFONT + str + "</font>";
	QMetaObject::invokeMethod(this, "append", Qt::QueuedConnection, Q_ARG(QString, text));
	// 向log文件输出
	if (IsOutPutLogFile == false)
		return;
	text = "[Error]" + QString("(") + current_date_time + QString(")");
	text += ":   ";
	text += str;
	out << text;
	out << "\n";
}