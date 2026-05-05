#!/bin/bash
# Pfad: ~/duckierace_groupe1/start_duckie.sh

# 1. Abfrage des Bot-Namens
echo -n "🤖 Welchen Duckiebot verwenden wir? (Enter für 'tick'): "
read USER_INPUT

if [ -z "$USER_INPUT" ]; then
    BOT_NAME="tick"
else
    BOT_NAME=$USER_INPUT
fi

echo "🚀 Starte 4 Split-Terminals für Duckiebot '$BOT_NAME'..."

# 2. Alte Sitzung schließen
tmux kill-session -t duckie_session 2>/dev/null

# 3. Neue Sitzung erstellen
tmux new-session -d -s duckie_session

# 4. Der Befehl mit dem dynamischen Namen
NET_CMD="source ~/duckierace_groupe1/netzwerk.sh $BOT_NAME"

# Pane 1 (oben links)
tmux send-keys -t duckie_session "$NET_CMD" C-m

# Pane 2 (oben rechts)
tmux split-window -h
tmux send-keys -t duckie_session "$NET_CMD" C-m

# Pane 3 (unten links)
tmux select-pane -t 0
tmux split-window -v
tmux send-keys -t duckie_session "$NET_CMD" C-m

# Pane 4 (unten rechts)
tmux select-pane -t 2
tmux split-window -v
tmux send-keys -t duckie_session "$NET_CMD" C-m

# 5. Sitzung anzeigen
tmux attach-session -t duckie_session