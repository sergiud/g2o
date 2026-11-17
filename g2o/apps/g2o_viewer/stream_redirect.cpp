// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "stream_redirect.h"

#include <QPlainTextEdit>
#include <QString>

#include <iostream>
using namespace std;

StreamRedirect::StreamRedirect(std::ostream &stream, QPlainTextEdit* te):
  _stream(stream), _te(te)
{
  _old_buf = stream.rdbuf();
  _stream.rdbuf(this);
}

StreamRedirect::~StreamRedirect()
{
  if (!_buffer.empty())
    xsputn(_buffer.c_str(), _buffer.size());
  _stream.rdbuf(_old_buf);
}

std::char_traits<char>::int_type StreamRedirect::overflow(int_type v)
{
  _mutex.lock();
  if (v == '\n') {
    _te->appendPlainText(QString::fromLatin1(_buffer.c_str(), _buffer.size()));
    _buffer.clear();
  }
  else
    _buffer.push_back(v);

  _mutex.unlock();
  return v;
}

std::streamsize StreamRedirect::xsputn(const char *p, std::streamsize n)
{
  _mutex.lock();
  _buffer.append(p, p + n);

  std::string::size_type pos = 0;
  while (1) {
    pos = _buffer.find('\n');
    if (pos != std::string::npos) {
      _te->appendPlainText(QString::fromLatin1(_buffer.c_str(), pos));
      _buffer.erase(_buffer.begin(), _buffer.begin() + pos + 1);
    } else
      break;
  }

  _mutex.unlock();
  return n;
}
