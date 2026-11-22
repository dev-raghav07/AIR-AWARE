#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_FLIGHTS 10
#define ID_LEN 10
#define AIRLINE_LEN 30
#define TEMP_BUF_LEN 64
#define LINE_BUF_LEN 256
#define EMERGENCY_FUEL_THRESHOLD 20.0
#define DISTANCE_THRESHOLD 2.0     
#define ALTITUDE_THRESHOLD 1000.0  
#define DATA_FILE "airspace_db.txt"
#define PI 3.14159265358979323846

typedef struct {
    double latitude;
    double longitude;
} Coordinate;

typedef struct {
    char flight_id[ID_LEN];
    char airline[AIRLINE_LEN];
    double altitude;
    double speed;
    double heading;
    double fuel;
    Coordinate pos;   
} Flight;

void display_airspace(const Flight *list, int count);
void register_flight(Flight *f, const Flight *list, int count);
void update_telemetry(Flight *f);
int check_emergencies(Flight *list, int count, Flight **alerts);
void check_collisions(const Flight *list, int count);
void save_airspace_to_disk(const Flight *list, int count);
int load_airspace_from_disk(Flight *list);
void simulate_time_step(Flight *list, int count);
void clear_input_buffer(void);
double read_double(const char *prompt);
void read_flight_id(char *buffer, size_t size);
double normalize_heading(double deg);
void clamp_coordinates(Coordinate *pos);

int main(void) {
    Flight airspace[MAX_FLIGHTS];
    int flight_count = 0; 
    
    int choice;
    char target_id[ID_LEN];
    int found;

    Flight *emergency_tracker[MAX_FLIGHTS];
    int emergency_count = 0;

    flight_count = load_airspace_from_disk(airspace);

    while (1) {
        printf("\n============================================\n");
        printf("      AIR TRAFFIC CONTROL RADAR SYSTEM      \n");
        printf("============================================\n");
        printf("1. View Active Airspace Radar Sweep\n");
        printf("2. Register New Incoming Flight\n");
        printf("3. Update Flight Telemetry\n");
        printf("4. Scan for Emergency / Low-Fuel Hazards\n");
        printf("5. Run Collision Warning System Scan\n");
        printf("6. Remove Flight (Handover / Out of Sector)\n");
        printf("7. Manually Save Airspace Data Matrix to Disk\n");
        printf("8. Simulate Airspace Time Step (10-Min Prediction Jump)\n");
        printf("9. Exit System (With Auto-Save)\n");
        printf("Enter your choice (1-9): ");
        
        if (scanf("%d", &choice) != 1) {
            printf("[ERROR] Invalid input. Please enter a number from 1 to 9.\n");
            clear_input_buffer();
            continue;
        }
        clear_input_buffer();

        switch (choice) {
            case 1:
                display_airspace(airspace, flight_count);
                break;

            case 2:
                if (flight_count >= MAX_FLIGHTS) {
                    printf("[ERROR] Airspace sector matrix is completely full!\n");
                } else {
                    register_flight(&airspace[flight_count], airspace, flight_count);
                    flight_count++;
                    printf("[OK] Flight successfully registered into current sector.\n");
                    save_airspace_to_disk(airspace, flight_count);
                }
                break;

            case 3:
                printf("Enter Flight ID to update: ");
                read_flight_id(target_id, sizeof(target_id));
                found = 0;
                
                for (int i = 0; i < flight_count; i++) {
                    if (strcmp(airspace[i].flight_id, target_id) == 0) {
                        update_telemetry(&airspace[i]);
                        found = 1;
                        save_airspace_to_disk(airspace, flight_count);
                        break;
                    }
                }
                if (!found) printf("[ERROR] Flight ID %s not found in active grid.\n", target_id);
                break;

            case 4:
                emergency_count = check_emergencies(airspace, flight_count, emergency_tracker);
                
                printf("\n--- CRITICAL RADAR SCAN RESULTS ---\n");
                if (emergency_count == 0) {
                    printf("[OK] Status Normal: No active critical conditions reported.\n");
                } else {
                    printf("[WARNING] Priority clearance required for %d aircraft:\n", emergency_count);
                    for (int i = 0; i < emergency_count; i++) {
                        printf(" -> ALERT: Flight %s (%s) | Fuel Level: %.1f%%\n", 
                               emergency_tracker[i]->flight_id, 
                               emergency_tracker[i]->airline, 
                               emergency_tracker[i]->fuel);
                    }
                }
                break;

            case 5:
                check_collisions(airspace, flight_count);
                break;

            case 6:
                printf("Enter Flight ID leaving sector: ");
                read_flight_id(target_id, sizeof(target_id));
                found = 0;

                for (int i = 0; i < flight_count; i++) {
                    if (strcmp(airspace[i].flight_id, target_id) == 0) {
                        found = 1;
                        for (int j = i; j < flight_count - 1; j++) {
                            airspace[j] = airspace[j + 1];
                        }
                        flight_count--; 
                        printf("[INFO] Flight %s handoff complete. Removed from sector.\n", target_id);
                        save_airspace_to_disk(airspace, flight_count);
                        break; 
                    }
                }
                if (!found) printf("[ERROR] Flight ID %s not found in active grid.\n", target_id);
                break;

            case 7:
                save_airspace_to_disk(airspace, flight_count);
                printf("[SAVED] Airspace matrix manually saved to %s.\n", DATA_FILE);
                break;

            case 8:
                simulate_time_step(airspace, flight_count);
                save_airspace_to_disk(airspace, flight_count);
                break;

            case 9:
                printf("Saving system state... ");
                save_airspace_to_disk(airspace, flight_count);
                printf("Shutting down ATC Radar Subroutines. Safe clear skies!\n");
                return 0;

            default:
                printf("[ERROR] Out of range selection. Please input a number from 1 to 9.\n");
        }
    }
    return 0;
}

void clear_input_buffer(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

void read_flight_id(char *buffer, size_t size) {
    char temp[TEMP_BUF_LEN];
    while (1) {
        if (fgets(temp, sizeof(temp), stdin) == NULL) {
            buffer[0] = '\0';
            return;
        }
        if (strchr(temp, '\n') == NULL) {
            clear_input_buffer();
            printf("[ERROR] Flight ID too long (max %zu chars). Try again: ", size - 1);
            continue;
        }
        temp[strcspn(temp, "\n")] = '\0';
        if (strlen(temp) == 0 || strlen(temp) >= size) {
            printf("[ERROR] Flight ID must be 1 to %zu characters. Try again: ", size - 1);
            continue;
        }
        for (int i = 0; temp[i] != '\0'; i++) {
            if (temp[i] == '|') temp[i] = '-';
        }
        strcpy(buffer, temp);
        return;
    }
}

double read_double(const char *prompt) {
    double val;
    while (1) {
        printf("%s", prompt);
        if (scanf("%lf", &val) == 1) {
            clear_input_buffer();
            return val;
        }
        printf("[ERROR] Invalid number. Please enter a valid numeric value.\n");
        clear_input_buffer();
    }
}

double normalize_heading(double deg) {
    deg = fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

void clamp_coordinates(Coordinate *pos) {
    if (pos->latitude > 90.0) pos->latitude = 90.0;
    if (pos->latitude < -90.0) pos->latitude = -90.0;

    pos->longitude = fmod(pos->longitude + 180.0, 360.0);
    if (pos->longitude < 0.0) pos->longitude += 360.0;
    pos->longitude -= 180.0;
}
void display_airspace(const Flight *list, int count) {
    if (count == 0) {
        printf("\n[RADAR] No flights currently tracked within this airspace sector.\n");
        return;
    }
    printf("\n%-10s %-15s %-11s %-9s %-9s %-7s %-16s\n", 
           "FLIGHT ID", "AIRLINE", "ALT(ft)", "SPD(kt)", "HDG(deg)", "FUEL", "COORDINATES");
    printf("-----------------------------------------------------------------------------------\n");
    for (int i = 0; i < count; i++) {
        printf("%-10s %-15s %-11.0f %-9.1f %-9.0f %-6.1f%% (%.2f, %.2f)\n", 
               list[i].flight_id, list[i].airline, list[i].altitude, 
               list[i].speed, list[i].heading, list[i].fuel, 
               list[i].pos.latitude, list[i].pos.longitude);
    }
}

void register_flight(Flight *f, const Flight *list, int count) {
    printf("\n--- Enter Flight Data ---\n");
    while (1) {
        printf("Flight ID (e.g. AI302): ");
        read_flight_id(f->flight_id, sizeof(f->flight_id));

        int duplicate = 0;
        for (int i = 0; i < count; i++) {
            if (strcmp(list[i].flight_id, f->flight_id) == 0) {
                duplicate = 1;
                break;
            }
        }
        if (!duplicate) break;
        printf("[WARNING] Flight ID %s already exists in sector. Enter a unique ID.\n", f->flight_id);
    }
    
    printf("Airline Name: ");
    if (fgets(f->airline, sizeof(f->airline), stdin) != NULL) {
        if (strchr(f->airline, '\n') == NULL) {
            clear_input_buffer();
        }
        f->airline[strcspn(f->airline, "\n")] = '\0';
    } else {
        strcpy(f->airline, "Unknown");
    }

    if (strlen(f->airline) == 0) {
        strcpy(f->airline, "Unknown");
    }

    for (int i = 0; f->airline[i] != '\0'; i++) {
        if (f->airline[i] == '|') f->airline[i] = '-';
    }

    f->altitude = read_double("Target Altitude (ft): ");
    if (f->altitude < 0.0) f->altitude = 0.0;

    f->speed = read_double("Cruising Speed (knots): ");
    if (f->speed < 0.0) f->speed = 0.0;

    f->heading = normalize_heading(read_double("Heading (0-360 deg): "));

    f->fuel = read_double("Fuel Remaining (0-100%%): ");
    if (f->fuel < 0.0) f->fuel = 0.0;
    if (f->fuel > 100.0) f->fuel = 100.0;

    f->pos.latitude = read_double("Latitude (-90 to 90): ");
    f->pos.longitude = read_double("Longitude (-180 to 180): ");
    clamp_coordinates(&f->pos);
}

void update_telemetry(Flight *f) {
    printf("\n--- Updating Telemetry for %s ---\n", f->flight_id);
    f->altitude = read_double("New Altitude (ft): ");
    if (f->altitude < 0.0) f->altitude = 0.0;

    f->speed = read_double("New Speed (knots): ");
    if (f->speed < 0.0) f->speed = 0.0;

    f->heading = normalize_heading(read_double("New Heading (0-360 deg): "));

    f->fuel = read_double("New Fuel Level (0-100%%): ");
    if (f->fuel < 0.0) f->fuel = 0.0;
    if (f->fuel > 100.0) f->fuel = 100.0;

    f->pos.latitude = read_double("New Latitude (-90 to 90): ");
    f->pos.longitude = read_double("New Longitude (-180 to 180): ");
    clamp_coordinates(&f->pos);
    printf("[OK] Telemetry updated for %s.\n", f->flight_id);
}

int check_emergencies(Flight *list, int count, Flight **alerts) {
    int alert_count = 0;
    for (int i = 0; i < count; i++) {
        if (list[i].fuel <= EMERGENCY_FUEL_THRESHOLD) {
            alerts[alert_count++] = &list[i];
        }
    }
    return alert_count;
}

void check_collisions(const Flight *list, int count) {
    int hazards = 0;
    printf("\n--- COLLISION AVOIDANCE SCAN ---\n");
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            double alt_diff = fabs(list[i].altitude - list[j].altitude);
            double dlat = list[i].pos.latitude - list[j].pos.latitude;
            double dlon = list[i].pos.longitude - list[j].pos.longitude;
            double dist = sqrt(dlat * dlat + dlon * dlon);

            if (alt_diff < ALTITUDE_THRESHOLD && dist < DISTANCE_THRESHOLD) {
                printf("[COLLISION WARNING] %s and %s | Dist: %.2f deg | Alt Diff: %.0f ft\n",
                       list[i].flight_id, list[j].flight_id, dist, alt_diff);
                hazards++;
            }
        }
    }
    if (hazards == 0) {
        printf("[OK] No separation violations detected.\n");
    }
}

void simulate_time_step(Flight *list, int count) {
    if (count == 0) {
        printf("[INFO] No active flights to simulate.\n");
        return;
    }
    double hours = 10.0 / 60.0;
    for (int i = 0; i < count; i++) {
        double distance_nm = list[i].speed * hours;
        double rad = list[i].heading * (PI / 180.0);

        list[i].pos.latitude += (distance_nm * cos(rad)) / 60.0;
        double lat_rad = list[i].pos.latitude * (PI / 180.0);
        double cos_lat = cos(lat_rad);
        if (fabs(cos_lat) > 0.0001) {
            list[i].pos.longitude += (distance_nm * sin(rad)) / (60.0 * cos_lat);
        }

        clamp_coordinates(&list[i].pos);
        list[i].fuel -= 3.0;
        if (list[i].fuel < 0.0) list[i].fuel = 0.0;
    }
    printf("[OK] All %d active flight vectors updated (+10 mins).\n", count);
    check_collisions(list, count);
}

void save_airspace_to_disk(const Flight *list, int count) {
    FILE *fp = fopen(DATA_FILE, "w");
    if (fp == NULL) {
        printf("[ERROR] Unable to open %s for saving.\n", DATA_FILE);
        return;
    }
    fprintf(fp, "%d\n", count);
    for (int i = 0; i < count; i++) {
        fprintf(fp, "%s|%s|%.2f|%.2f|%.2f|%.2f|%.4f|%.4f\n",
                list[i].flight_id,
                list[i].airline,
                list[i].altitude,
                list[i].speed,
                list[i].heading,
                list[i].fuel,
                list[i].pos.latitude,
                list[i].pos.longitude);
    }
    fclose(fp);
}

int load_airspace_from_disk(Flight *list) {
    FILE *fp = fopen(DATA_FILE, "r");
    if (fp == NULL) {
        return 0;
    }
    char line[LINE_BUF_LEN];
    if (fgets(line, sizeof(line), fp) == NULL) {
        fclose(fp);
        return 0;
    }
    int count = 0;
    if (sscanf(line, "%d", &count) != 1 || count <= 0) {
        fclose(fp);
        return 0;
    }
    if (count > MAX_FLIGHTS) count = MAX_FLIGHTS;
    int loaded = 0;
    for (int i = 0; i < count && fgets(line, sizeof(line), fp) != NULL; i++) {
        Flight temp;
        if (sscanf(line, "%9[^|]|%29[^|]|%lf|%lf|%lf|%lf|%lf|%lf",

            
                   temp.flight_id,
                   temp.airline,
                   &temp.altitude,
                   &temp.speed,
                   &temp.heading,
                   &temp.fuel,
                   &temp.pos.latitude,
                   &temp.pos.longitude) == 8) {
            temp.heading = normalize_heading(temp.heading);
            clamp_coordinates(&temp.pos);
            if (temp.fuel < 0.0) temp.fuel = 0.0;
            if (temp.fuel > 100.0) temp.fuel = 100.0;
            list[loaded++] = temp;
        }
    }
    fclose(fp);
    if (loaded > 0) {
        printf("[DB] Loaded %d saved flight record(s) from %s.\n", loaded, DATA_FILE);
    }
    return loaded;
}