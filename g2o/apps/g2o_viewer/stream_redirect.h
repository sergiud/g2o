// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef G2O_STREAM_REDIRECT_H
#define G2O_STREAM_REDIRECT_H

#include <QMutex>
#include <iostream>
#include <streambuf>
#include <string>

#include <g2o/viewer/g2o_viewer_api.h>

class QPlainTextEdit;

/**
 * \brief redirect a stream to a QPlainTextEdit
 */
class G2O_VIEWER_API StreamRedirect : public std::basic_streambuf<char>
{
  public:
    typedef std::char_traits<char>::int_type int_type;

  public:
    StreamRedirect(std::ostream &stream, QPlainTextEdit* te);
    ~StreamRedirect();

  protected:
    virtual std::char_traits<char>::int_type overflow(int_type v);
    virtual std::streamsize xsputn(const char *p, std::streamsize n); 

  private:
    std::ostream& _stream;
    std::streambuf* _old_buf;
    std::string _buffer;
    QPlainTextEdit* _te;
    QMutex _mutex;
};

#endif
