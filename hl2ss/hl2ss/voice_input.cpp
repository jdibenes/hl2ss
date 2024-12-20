
#include <vector>
#include <queue>
#include "extended_execution.h"
#include "voice_input.h"
#include "lock.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.SpeechRecognition.h>

using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::SpeechRecognition;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

CRITICAL_SECTION g_lock; // DeleteCriticalSection
HANDLE g_event_completed = NULL; // CloseHandle
SpeechRecognizer g_recognizer = nullptr;
std::vector<winrt::hstring> g_commands;
std::queue<VoiceInput_Result> g_queue;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static uint32_t VoiceInput_FindID(winrt::hstring const& query)
{
    uint32_t index = 0;
    for (auto const& command : g_commands) { if (command == query) { return index; } else { ++index; } }
    return index;
}

// OK
static void VoiceInput_OnCompleted(SpeechContinuousRecognitionSession scrs, SpeechContinuousRecognitionCompletedEventArgs const& args)
{
    (void)scrs;
    (void)args;

    SetEvent(g_event_completed);
}

// OK
static void VoiceInput_OnResultGenerated(SpeechContinuousRecognitionSession scrs, SpeechContinuousRecognitionResultGeneratedEventArgs const& args)
{
    (void)scrs;

    VoiceInput_Result result;

    result.Index           = VoiceInput_FindID(args.Result().Text());
    result.Confidence      = static_cast<uint32_t>(args.Result().Confidence());
    result.PhraseDuration  = args.Result().PhraseDuration().count();
    result.PhraseStartTime = args.Result().PhraseStartTime().time_since_epoch().count();
    result.RawConfidence   = args.Result().RawConfidence();

    CriticalSection cs(&g_lock);
    g_queue.push(result);
}

// OK
void VoiceInput_Startup()
{
    InitializeCriticalSection(&g_lock);
    g_event_completed = CreateEvent(NULL, TRUE, TRUE, NULL);
}

// OK
void VoiceInput_Cleanup()
{
    CloseHandle(g_event_completed);
    g_event_completed = NULL;
    DeleteCriticalSection(&g_lock);
}

// OK
void VoiceInput_Open()
{
    g_recognizer = SpeechRecognizer();
    g_recognizer.ContinuousRecognitionSession().ResultGenerated(VoiceInput_OnResultGenerated);
    g_recognizer.ContinuousRecognitionSession().Completed(VoiceInput_OnCompleted);
}

// OK
void VoiceInput_Close()
{
    g_recognizer.Close();
    g_recognizer = nullptr;
}

// OK
bool VoiceInput_RegisterCommands(std::vector<winrt::hstring> const& strings)
{
    SpeechRecognitionListConstraint srlc = SpeechRecognitionListConstraint(strings);
    g_recognizer.Constraints().Clear();
    g_commands.clear();
    g_recognizer.Constraints().Append(srlc);
    g_commands.insert(g_commands.end(), strings.begin(), strings.end());
    SpeechRecognitionCompilationResult result = g_recognizer.CompileConstraintsAsync().get();
    return result.Status() == SpeechRecognitionResultStatus::Success;
}

// OK
void VoiceInput_Start()
{
    ResetEvent(g_event_completed);
    try
    {
    Cleaner log_error_microphone([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedMicrophone); });
    g_recognizer.ContinuousRecognitionSession().StartAsync().get();
    log_error_microphone.Set(false);
    }
    catch (...)
    {
    SetEvent(g_event_completed);
    }
}

// OK
void VoiceInput_Stop()
{
    try
    {
    g_recognizer.ContinuousRecognitionSession().StopAsync().get();
    WaitForSingleObject(g_event_completed, INFINITE);
    }
    catch(...)
    {
    }

    CriticalSection cs(&g_lock);
    g_queue = {};
}

// OK
bool VoiceInput_Status()
{
    return WaitForSingleObject(g_event_completed, 0) == WAIT_TIMEOUT;
}

// OK
uint32_t VoiceInput_GetCount()
{
    CriticalSection cs(&g_lock);
    return static_cast<uint32_t>(g_queue.size());
}

// OK
VoiceInput_Result VoiceInput_Pop()
{
    CriticalSection cs(&g_lock);
    VoiceInput_Result result = g_queue.front();
    g_queue.pop();
    return result;
}
