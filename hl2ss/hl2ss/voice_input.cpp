
#include <vector>
#include <queue>
#include "lock.h"
#include "voice_input.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.SpeechRecognition.h>

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
void VoiceInput_Initialize()
{
    InitializeCriticalSection(&g_lock);
    g_event_completed = CreateEvent(NULL, TRUE, TRUE, NULL);
}

// OK
static uint32_t VoiceInput_FindID(winrt::hstring const& query)
{
    uint32_t index = 0;
    for (auto const& command : g_commands) { if (command == query) { return index; } else { ++index; } }
    return index;
}

// OK
static void VoiceInput_Completed(SpeechContinuousRecognitionSession scrs, SpeechContinuousRecognitionCompletedEventArgs const& args)
{
    (void)scrs;
    (void)args;
    SetEvent(g_event_completed);
}

// OK
static void VoiceInput_ResultGenerated(SpeechContinuousRecognitionSession scrs, SpeechContinuousRecognitionResultGeneratedEventArgs const& args)
{
    (void)scrs;
    VoiceInput_Result result;
    result.Index = VoiceInput_FindID(args.Result().Text());
    result.Confidence = (uint32_t)args.Result().Confidence();
    result.PhraseDuration = args.Result().PhraseDuration().count();
    result.PhraseStartTime = args.Result().PhraseStartTime().time_since_epoch().count();
    result.RawConfidence = args.Result().RawConfidence();
    CriticalSection cs(&g_lock);
    g_queue.push(result);
}

// OK
void VoiceInput_CreateRecognizer()
{
    if (g_recognizer) { g_recognizer.Close(); }
    g_recognizer = SpeechRecognizer();
    g_recognizer.ContinuousRecognitionSession().ResultGenerated(VoiceInput_ResultGenerated);
    g_recognizer.ContinuousRecognitionSession().Completed(VoiceInput_Completed);
}

// OK
bool VoiceInput_RegisterCommands(std::vector<winrt::hstring> const& strings, bool clear)
{
    SpeechRecognitionListConstraint srlc = SpeechRecognitionListConstraint(strings);
    if (clear)
    { 
        g_recognizer.Constraints().Clear();
        g_commands.clear();
    }
    g_recognizer.Constraints().Append(srlc);
    g_commands.insert(g_commands.end(), strings.begin(), strings.end());
    SpeechRecognitionCompilationResult result = g_recognizer.CompileConstraintsAsync().get();
    return result.Status() == SpeechRecognitionResultStatus::Success;
}

// OK
void VoiceInput_Start()
{
    ResetEvent(g_event_completed);
    g_recognizer.ContinuousRecognitionSession().StartAsync().get();
}

// OK
void VoiceInput_Stop()
{
    g_recognizer.ContinuousRecognitionSession().StopAsync().get();
    WaitForSingleObject(g_event_completed, INFINITE);
}

// OK
size_t VoiceInput_GetCount()
{
    CriticalSection cs(&g_lock);
    return g_queue.size();
}

// OK
VoiceInput_Result VoiceInput_Pop()
{
    CriticalSection cs(&g_lock);
    VoiceInput_Result result = g_queue.front();
    g_queue.pop();
    return result;
}

// OK
void VoiceInput_Clear()
{
    CriticalSection cs(&g_lock);
    g_queue = {};
}

// OK
bool VoiceInput_IsRunning()
{
    return WaitForSingleObject(g_event_completed, 0) == WAIT_TIMEOUT;
}
