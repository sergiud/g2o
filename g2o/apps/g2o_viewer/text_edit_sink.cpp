// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "text_edit_sink.h"

#include <ng-log/experimental/setup.h>

#include <QMetaObject>
#include <QPlainTextEdit>
#include <QString>

TextEditSink::TextEditSink(QPlainTextEdit* textEdit) : _textEdit(textEdit) {
  g2o::ngx::AddSink(*this);
}

TextEditSink::~TextEditSink() { g2o::ngx::RemoveSink(*this); }

void TextEditSink::Send(const g2o::ngx::Record& record) {
  const QString text = QString::fromUtf8(
      record.text().data(), static_cast<qsizetype>(record.text().size()));
  QMetaObject::invokeMethod(_textEdit, "appendPlainText", Qt::AutoConnection,
                            Q_ARG(QString, text));
}
