!/bin/bash



# Configuration

PORT="/dev/ttyUSB0"

BAUD="115200"



# Initialize serial port settings

# We use 'raw' to ensure no byte translation happens

stty -F $PORT $BAUD raw -echo -echoe -echok -echoctl -echoke



echo "Starting LoRa RAW Monitor on $PORT..."

echo "Format: [HEX] [ASCII Equivalent]"

echo "Press [CTRL+C] to stop."



# --- RX Section ---

# We use 'od' (octal dump) or 'hexdump' to format the binary data coming in

# This mimics your Arduino's printRawSerial output

( stty -F $PORT $BAUD raw; od -An -v -t x1 -t c $PORT ) &

RX_PID=$!



# Cleanup on exit

trap "kill $RX_PID; exit" INT TERM EXIT



# --- TX Section ---

while true

do

   # The Arduino expects to see "AAA" (which is 0x41 0x41 0x41)

   echo -ne "AAAAAAAAAAAAAAAAAAAAAAA" > $PORT

    

   # We print the timestamp locally so you know when the script sent data

   echo -e "\n[$(date +%T)] Sent: AAAAAAAAAA"



   # Match your Arduino's 2-second interval or keep it at 5s

   sleep 5

done
