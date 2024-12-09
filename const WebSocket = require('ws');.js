const WebSocket = require('ws');
const sqlite3 = require('sqlite3').verbose();

// Initialize SQLite database
const db = new sqlite3.Database('esp_data.db', (err) => {
    if (err) {
        console.error('Error opening database:', err.message);
    } else {
        console.log('Connected to SQLite database.');
    }
});

// Create WebSocket server
const server = new WebSocket.Server({ port: 8266 });

server.on('connection', (socket) => {
    console.log('New client connected.');

    socket.on('message', (message) => {
        console.log('Received:', message);

        // Parse the incoming JSON data
        try {
            const data = JSON.parse(message);
            const {
                protocol,
                voltage,
                current,
                power,
                total_energy,
                run_time,
                transmission_attempts,
                transmission_failures,
            } = data;

            // Insert data into SQLite database
            const query = `
                INSERT INTO energy_data (protocol, voltage, current, power, total_energy, run_time, transmission_attempts, transmission_failures)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            `;
            db.run(
                query,
                [
                    protocol,
                    voltage,
                    current,
                    power,
                    total_energy,
                    run_time,
                    transmission_attempts,
                    transmission_failures,
                ],
                (err) => {
                    if (err) {
                        console.error('Error inserting data:', err.message);
                    } else {
                        console.log('Data inserted successfully.');
                    }
                }
            );
        } catch (err) {
            console.error('Error parsing message:', err.message);
        }
    });

    socket.on('close', () => {
        console.log('Client disconnected.');
    });
});

console.log('WebSocket server is running on ws://0.0.0.0:8266');
