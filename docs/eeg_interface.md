# Proposed interface with the EEG team (to discuss)

Goal: agree on what the classifier sends, so the shared-autonomy layer and the simulator can be built against it now.

**Message (one per classifier decision):**
| Field | Example | Notes |
|---|---|---|
| `label` | `left`, `right`, `forward`, `backward`, `rest` | `rest` = no command / uncertain |
| `confidence` | `0.82` | probability of `label`, 0–1 |
| `probs` | `{left: 0.82, right: 0.05, ...}` | full distribution, if available (most useful for shared autonomy) |
| `stamp` | seconds | when the EEG window ended |

**Transport options:** a ROS 2 topic (`/eeg/intent`), or JSON over UDP/websocket if the EEG code doesn't use ROS.

**Questions for the team:** decisions per second and latency; expected accuracy per class; whether `backward`
is a real class (motor imagery usually uses left hand, right hand, feet, tongue); whether there is an idle / rest state.

Until the real classifier is ready, the simulator will generate this message from keyboard input with
deliberate random errors at a chosen accuracy.
