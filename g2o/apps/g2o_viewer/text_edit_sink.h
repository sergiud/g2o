// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_APPS_G2O_VIEWER_TEXT_EDIT_SINK_H
#define G2O_APPS_G2O_VIEWER_TEXT_EDIT_SINK_H

#include <g2o/stuff/logger.h>
#include <g2o/viewer/g2o_viewer_api.h>
#include <ng-log/experimental/sink.h>

class QPlainTextEdit;

/**
 * Shows the messages that g2o logs in a text edit while the sink exists.
 * Messages logged on other threads are appended on the thread of the text
 * edit.
 */
class G2O_VIEWER_API TextEditSink final : public g2o::ngx::Sink {
 public:
  explicit TextEditSink(QPlainTextEdit* textEdit);
  ~TextEditSink();

  TextEditSink(const TextEditSink&) = delete;
  TextEditSink& operator=(const TextEditSink&) = delete;

  void Send(const g2o::ngx::Record& record) override;

 private:
  QPlainTextEdit* _textEdit;
};

#endif
