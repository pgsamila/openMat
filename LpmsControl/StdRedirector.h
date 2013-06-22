/****************************************************************************
**
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** OpenMAT is free software: you can redistribute it and/or modify it under 
** the terms of the GNU General Public License as published by the Free 
** Software Foundation, either version 3 of the License, or (at your option) 
** any later version.
** 
** OpenMAT is distributed in the hope that it will be useful, but WITHOUT 
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
** FITNESS FOR A PARTICULAR PURPOSE. See the GNU \ General Public License 
** for more details.
** 
** You should have received a copy of the GNU General Public License along 
** with the OpenMAT library. If not, see <http://www.gnu.org/licenses/>.
**
****************************************************************************/

#ifndef STD_REDIRECTOR
#define STD_REDIRECTOR

#include <QWidget>
#include <QTextEdit>
#include <QString>
#include <QVBoxLayout>
#include <QTimer>
#include <QMutex>
#include <QScrollBar>

#include <iostream>
#include <string>

using namespace std;

template<class Elem = char, class Tr = std::char_traits<Elem>>
 
/*!
	\brief	Redirects the output from the console to a ConsoleWindow.
*/
class StdRedirector : public std::basic_streambuf<Elem, Tr>
{
	typedef void (*pfncb) ( const Elem*, std::streamsize _Count, void* pUsrData );

	public:
		StdRedirector(std::ostream& a_Stream, pfncb a_Cb, void* a_pUsrData) :
			m_Stream(a_Stream),
			m_pCbFunc(a_Cb),
			m_pUserData(a_pUsrData)
		{
			m_pBuf = m_Stream.rdbuf(this);
		}

		~StdRedirector()
		{
			m_Stream.rdbuf(m_pBuf);
		}

		std::streamsize xsputn(const Elem* _Ptr, std::streamsize _Count)
		{
			m_pCbFunc(_Ptr, _Count, m_pUserData);
			return _Count;
		}

		typename Tr::int_type overflow(typename Tr::int_type v)
		{
			Elem ch = Tr::to_char_type(v);
			m_pCbFunc(&ch, 1, m_pUserData);
			return Tr::not_eof(v);
		}

	protected:
		std::basic_ostream<Elem, Tr>& m_Stream;
		std::streambuf* m_pBuf;
		pfncb m_pCbFunc;
		void* m_pUserData;
};

/*!
	\brief	A window that contains the output of the system console.
*/
class ConsoleWindow : public QWidget
{
Q_OBJECT

	public:
		ConsoleWindow(QWidget *parent = 0);
		~ConsoleWindow();
		
	public slots:
		void updateTimer(void);
	
	public:
		QTextEdit *consoleBox;
		StdRedirector<> *stdRedirector;
		string bufferString;
};
	
#endif
