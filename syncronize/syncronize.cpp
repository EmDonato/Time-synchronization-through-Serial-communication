#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <arpa/inet.h>  // Per ntohl()
using namespace std;
using namespace chrono;

// Funzione per ottenere il tempo corrente in secondi e nanosecondi
void get_current_time(uint32_t &sec, uint32_t &nanosec) {
    auto now = steady_clock::now();
    auto duration = now.time_since_epoch();
    auto sec_since_epoch = duration_cast<seconds>(duration);
    auto ns_since_epoch = duration_cast<nanoseconds>(duration) - duration_cast<seconds>(duration);

    sec = static_cast<uint32_t>(sec_since_epoch.count());
    nanosec = static_cast<uint32_t>(ns_since_epoch.count());
    cout << "[DEBUG] Tempo corrente ottenuto: " << sec << " s, " << nanosec << " ns." << endl;
}

// Funzione per configurare la porta seriale
int configure_serial_port(const char *port) {
    cout << "[DEBUG] Apertura della porta seriale: " << port << endl;
    int serial_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        cerr << "[ERRORE] Impossibile aprire la porta seriale." << endl;
        return -1;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(serial_fd, TCSANOW, &options);

    cout << "[DEBUG] Porta seriale configurata correttamente." << endl;
    return serial_fd;
}

// Funzione per inviare il timestamp all'ESP32
void send_timestamp_to_esp32(int serial_fd) {
    uint32_t sec, nanosec;
    get_current_time(sec, nanosec);

    cout << "[DEBUG] Invio del timestamp all'ESP32..." << endl;
    if (write(serial_fd, &sec, sizeof(sec)) == sizeof(sec) &&
        write(serial_fd, &nanosec, sizeof(nanosec)) == sizeof(nanosec)) {
        cout << "[INFO] Timestamp inviato: " << sec << " s, " << nanosec << " ns." << endl;
    } else {
        cerr << "[ERRORE] Fallito l'invio del timestamp!" << endl;
    }
}

// Funzione per ricevere il segnale di avvio dall'ESP32
void receive_start_signal_from_esp32(int serial_fd) {
    uint8_t signal;
    cout << "[DEBUG] In attesa del segnale di avvio dall'ESP32..." << endl;
    while (true) {
        int bytesRead = read(serial_fd, &signal, sizeof(signal));
        if (bytesRead == sizeof(signal)) {
            cout << "[DEBUG] Dato ricevuto: " << static_cast<int>(signal) << endl;
            if (signal == 1) {
                cout << "[INFO] Segnale di avvio ricevuto!" << endl;
                break;
            }
        } else {
            cout << "[DEBUG] Nessun dato ricevuto, riprovo..." << endl;
            usleep(500000);  // Aspetta 500 ms prima di riprovare
        }
    }
}

int main() {
    const char *port = "/dev/ttyUSB0";
    int serial_fd = configure_serial_port(port);
    if (serial_fd == -1) {
        return -1;
    }

    cout << "[INFO] Connessione alla porta seriale stabilita." << endl;
    receive_start_signal_from_esp32(serial_fd);

    cout << "[INFO] Inizio sincronizzazione dei timestamp." << endl;
    while (true) {
        send_timestamp_to_esp32(serial_fd);
        cout << "[DEBUG] Attesa di 10 secondi prima del prossimo invio..." << endl;
        sleep(10);
    }

    close(serial_fd);
    return 0;
}


