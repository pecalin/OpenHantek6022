// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <memory>
#include <vector>

#include "hantekprotocol/types.h"

class DsoWidget;
class HorizontalDock;
class PPresult;
class TriggerDock;
class VoltageDock;
struct DsoSettingsScope;

/// \brief Finds a good display setup for the active channels, like the "Autoset" button of a real scope.
/// Starts with the least sensitive gain, measures the signals and adapts gain, offset, timebase,
/// trigger source and trigger level step by step, each step is checked with samples taken with the new settings.
class AutoSet : public QObject {
    Q_OBJECT

  public:
    AutoSet( DsoSettingsScope *scope, VoltageDock *voltageDock, HorizontalDock *horizontalDock, TriggerDock *triggerDock,
             DsoWidget *dsoWidget, QObject *parent = nullptr );

    /// \brief Start a new autoset sequence, it continues with the following results.
    void start();
    bool isActive() const { return step != Step::IDLE; }

    /// \brief Process new samples, adapt the settings and wait for samples taken with these settings.
    void input( const std::shared_ptr< PPresult > &result );

  signals:
    void statusMessage( const QString &message, int timeout ); ///< Progress and result of the autoset

  private:
    enum class Step { IDLE, COARSE, FINE };

    /// The smallest V/div step that shows the signal within span divs around the screen position center
    unsigned fitGain( ChannelID channel, double vmin, double vmax, double span, double center ) const;
    /// Ignore samples until the new settings are active, longer for slow timebases
    void waitForSamples( double timebase );
    void finish( const QString &message );

    DsoSettingsScope *scope;
    VoltageDock *voltageDock;
    HorizontalDock *horizontalDock;
    TriggerDock *triggerDock;
    DsoWidget *dsoWidget;

    Step step = Step::IDLE;
    unsigned iteration = 0;
    unsigned emptyResults = 0;
    std::vector< ChannelID > channels; ///< the used voltage channels CH1, CH2
    QElapsedTimer settleTimer;
    qint64 settleTime = 0; ///< ms
};
