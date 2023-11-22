DEMO VBAN RtAudio client for network and pipes/fifos

WARNING!!! this is "quick and dirty" version, made for tests and trainings.
Clean versions (cli and gui) - coming soon!

usage: vban_rtaudio <args>

-m - mode: rx - net/fifo to audio, tx (default) - audio to net/fifo
-d - device name (for ALSA without "hw:", for others - see manuals)
-s - samplerate (default 48000)
-q - quantum, buffer size (Attention!!! Default is 128!!! Made for musicians.)
-c - number of channels
-n - redundancy 1 to 10 (in rx mode - as "net quality", it tx mode not implemented yet)
-f - format: 16, 24, 32f
-i - ip address or pipe name
-p - ip port (if 0 - pipe)
-s - Stream name, up to symbols
-b - backend (alsa, pulse, jack for linux; coreaudio, jack for mac; asio, wasapi for windows)
-h - show this help
