#include "MyThread.h"

wxThread::ExitCode MyThread::Entry()
{
    for (int n=0; n<3000; n++)
    {
        this->Sleep(2000);
 
        // notify the main thread
        wxCommandEvent event( wxEVT_COMMAND_TEXT_UPDATED, NUMBER_UPDATE_ID );
        event.SetInt(n);  // pass some data along the event, a number in this case
        m_parent->GetEventHandler()->AddPendingEvent( event );
    }
    return 0;
}