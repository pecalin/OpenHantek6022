// SPDX-License-Identifier: GPL-2.0-or-later

#include <QDebug>
#include <algorithm>
#include <cmath>

#include "HorizontalDock.h"
#include "TriggerDock.h"
#include "VoltageDock.h"
#include "autoset.h"
#include "dsowidget.h"
#include "post/ppresult.h"
#include "scopesettings.h"
#include "viewconstants.h"

namespace {
const double coarseTimebase = 1e-3;  ///< 1 ms/div finds the frequency of most signals (~100 Hz .. ~100 kHz)
const double periodsOnScreen = 5;    ///< rounded down to the next 1-2-5 timebase step -> 2 .. 5 periods on screen
const double minPeriodicSpan = 0.5;  ///< smaller signals (divs) are treated as DC or noise without a frequency
const double usedArea = 0.75;        ///< part of its screen area that a channel shall fill
const unsigned maxIterations = 6;    ///< stop if the settings do not converge
const unsigned maxEmptyResults = 10; ///< stop if no samples arrive
} // namespace


AutoSet::AutoSet( DsoSettingsScope *scope, VoltageDock *voltageDock, HorizontalDock *horizontalDock, TriggerDock *triggerDock,
                  DsoWidget *dsoWidget, QObject *parent )
    : QObject( parent ), scope( scope ), voltageDock( voltageDock ), horizontalDock( horizontalDock ), triggerDock( triggerDock ),
      dsoWidget( dsoWidget ) {}


void AutoSet::start() {
    channels.clear();
    for ( ChannelID channel = 0; channel + 1 < scope->voltage.size(); ++channel ) // CH1, CH2 without MATH
        if ( scope->voltage[ channel ].used )
            channels.push_back( channel );
    if ( channels.empty() ) {
        finish( tr( "Autoset: switch on CH1 or CH2 first" ) );
        return;
    }
    if ( scope->verboseLevel > 1 )
        qDebug() << " AutoSet::start()" << channels.size() << "channels";
    emit statusMessage( tr( "Autoset ..." ), 0 );
    triggerDock->changeMode( Dso::TriggerMode::AUTO ); // show the trace also without trigger, leave ROLL and SINGLE
    // measure with the least sensitive gain first to avoid clipping
    for ( ChannelID channel : channels ) {
        voltageDock->changeGain( channel, unsigned( scope->gainSteps.size() - 1 ) );
        dsoWidget->changeOffset( channel, 0 );
    }
    const double timebase = horizontalDock->changeTimebase( coarseTimebase );
    step = Step::COARSE;
    iteration = 0;
    emptyResults = 0;
    waitForSamples( timebase );
}


void AutoSet::input( const std::shared_ptr< PPresult > &result ) {
    if ( step == Step::IDLE || settleTimer.elapsed() < settleTime )
        return; // not active or the samples may still be taken with the previous settings

    struct Measurement {
        ChannelID channel;
        double vmin;
        double vmax;
        double frequency;
        bool clipped;
    };
    std::vector< Measurement > measured;
    for ( ChannelID channel : channels ) {
        const DataChannel *data = result->data( channel );
        if ( !data || data->voltage.samples.empty() )
            continue;
        const auto minmax = std::minmax_element( data->voltage.samples.begin(), data->voltage.samples.end() );
        measured.push_back( { channel, *minmax.first, *minmax.second, data->frequency, !data->valid } );
    }
    if ( measured.empty() ) {
        if ( ++emptyResults > maxEmptyResults )
            finish( tr( "Autoset: no samples available" ) );
        return;
    }
    emptyResults = 0;

    bool changed = false;
    // vertical: each channel gets its own part of the screen, CH1 on top
    const double area = DIVS_VOLTAGE / double( channels.size() );
    const Measurement *trigger = nullptr;
    double triggerScore = -1;
    for ( const Measurement &m : measured ) {
        const size_t position = size_t( std::find( channels.begin(), channels.end(), m.channel ) - channels.begin() );
        const double center = DIVS_VOLTAGE / 2 - area * ( double( position ) + 0.5 );
        const unsigned currentIndex = scope->voltage[ m.channel ].gainStepIndex;
        unsigned gainIndex = fitGain( m.channel, m.vmin, m.vmax, area * usedArea, center );
        if ( m.clipped ) // the real signal is bigger than measured, use at least the next less sensitive gain
            gainIndex = std::max( gainIndex, std::min( currentIndex + 1, unsigned( scope->gainSteps.size() - 1 ) ) );
        if ( gainIndex != currentIndex ) {
            voltageDock->changeGain( m.channel, gainIndex );
            changed = true;
        }
        const double gain = scope->gain( m.channel ); // V/div
        dsoWidget->changeOffset( m.channel,
                                 qBound( -DIVS_VOLTAGE / 2, center - ( m.vmin + m.vmax ) / 2 / gain, DIVS_VOLTAGE / 2 ) );
        // trigger on the biggest periodic signal, else on the biggest signal
        const double span = ( m.vmax - m.vmin ) / gain;
        const double score = span + ( m.frequency > 0 && span >= minPeriodicSpan ? 2 * DIVS_VOLTAGE : 0 );
        if ( score > triggerScore ) {
            triggerScore = score;
            trigger = &m;
        }
    }

    // trigger in the middle of the signal
    triggerDock->changeSource( int( trigger->channel ) );
    dsoWidget->changeTriggerLevel( trigger->channel, ( trigger->vmin + trigger->vmax ) / 2 );

    // horizontal: show some periods of the trigger signal, keep the timebase for DC or noise
    double timebase = scope->horizontal.timebase;
    const bool periodic = trigger->frequency > 0 && ( trigger->vmax - trigger->vmin ) / scope->gain( trigger->channel ) >= minPeriodicSpan;
    if ( periodic ) {
        const double periods = trigger->frequency * timebase * DIVS_TIME;
        if ( periods < 2 || periods > 6 ) { // keep a good timebase, do not toggle between two steps
            const double newTimebase = horizontalDock->changeTimebase( periodsOnScreen / trigger->frequency / DIVS_TIME );
            changed |= std::fabs( newTimebase - timebase ) > timebase * 1e-6;
            timebase = newTimebase;
        }
    }
    if ( scope->verboseLevel > 1 )
        qDebug() << " AutoSet::input()" << int( step ) << iteration << "trigger CH" << trigger->channel + 1 << trigger->vmin
                 << trigger->vmax << trigger->frequency << "Hz, timebase" << timebase << "changed" << changed;

    // check the new settings with new samples, stop if they are stable
    if ( step == Step::FINE && !changed ) {
        finish( tr( "Autoset done" ) );
        return;
    }
    if ( ++iteration >= maxIterations ) {
        finish( tr( "Autoset done, the signal is not stable" ) );
        return;
    }
    step = Step::FINE;
    waitForSamples( timebase );
}


unsigned AutoSet::fitGain( ChannelID channel, double vmin, double vmax, double span, double center ) const {
    const double attn = scope->voltage[ channel ].probeAttn;
    for ( unsigned index = 0; index < scope->gainSteps.size(); ++index ) {
        const double gain = scope->gainSteps[ index ] * attn;       // V/div
        const double offset = center - ( vmin + vmax ) / 2 / gain; // screen offset that centers the signal
        if ( ( vmax - vmin ) / gain <= span && std::fabs( offset ) <= DIVS_VOLTAGE / 2 )
            return index;
    }
    return unsigned( scope->gainSteps.size() - 1 );
}


void AutoSet::waitForSamples( double timebase ) {
    // the new settings must reach the scope, then a complete new sample block (up to two screens) must arrive
    settleTime = 300 + qint64( 4 * 1000 * timebase * DIVS_TIME );
    settleTimer.start();
}


void AutoSet::finish( const QString &message ) {
    if ( scope->verboseLevel > 1 )
        qDebug() << " AutoSet::finish()" << message;
    step = Step::IDLE;
    emit statusMessage( message, 3000 );
}
