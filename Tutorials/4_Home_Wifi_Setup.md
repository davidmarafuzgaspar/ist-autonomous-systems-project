# Tutorial 4 — Home Wi-Fi Setup via SD Card

Configure the robot to connect to your home Wi-Fi by editing NetworkManager profiles on the SD card. No SSH changes are required on the robot after boot.

---

## Goal

1. Remove the SD card from the robot
2. Add a home Wi-Fi profile on your laptop
3. Set correct permissions and priorities
4. Boot at home and connect automatically

---

## Step 1 — Remove the SD card

Power off the robot and remove the microSD card. Insert it into your laptop using an adapter.

---

## Step 2 — Find the root filesystem partition

```bash
lsblk
```

Look for something like:

```text
sdb
├─sdb1  boot
└─sdb2  writable    ← Linux system (this one)
```

The mount path is usually:

```text
/media/$USER/writable
```

If it is not mounted automatically, mount `sdb2` (replace device name if different).

---

## Step 3 — Open the NetworkManager config folder

```bash
cd /media/$USER/writable/etc/NetworkManager/system-connections/
ls
```

You should see existing profiles, for example:

```text
Lab-Connection.nmconnection
Hotspot-Fallback.nmconnection
```

---

## Step 4 — Create the home Wi-Fi profile

Create a new file:

```bash
sudo nano HomeWiFi.nmconnection
```

Paste (edit `ssid` and `psk` for your home network):

```ini
[connection]
id=HomeWiFi
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=20

[wifi]
mode=infrastructure
ssid=Wi-Fi Repeater 2

[wifi-security]
key-mgmt=wpa-psk
psk=JQM259MW

[ipv4]
method=auto

[ipv6]
method=auto
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X` in nano).

---

## Step 5 — Fix file permissions (required)

NetworkManager ignores profiles with insecure permissions.

```bash
sudo chmod 600 HomeWiFi.nmconnection
sudo chown root:root HomeWiFi.nmconnection
```

---

## Step 6 — Adjust lab network priority (recommended)

Open the lab profile:

```bash
sudo nano Lab-Connection.nmconnection
```

Under `[connection]`, set or add:

```ini
autoconnect-priority=1
```

Resulting priority:

| Profile | Priority | When used |
|---------|----------|-----------|
| HomeWiFi | 20 | Preferred at home |
| Lab-Connection | 1 | Used at school if home SSID absent |

---

## Step 7 — Do not copy school static IP to home

The lab file may contain:

```ini
address1=192.168.28.68/24,192.168.28.1
```

Do not reuse this at home. It is a static school configuration.

At home, always use DHCP:

```ini
method=auto
```

Your home profile in Step 4 already uses `method=auto`.

---

## Step 8 — Safely eject the SD card

```bash
sync
sudo umount /media/$USER/writable
```

Remove the SD card from the laptop and insert it back into the robot.

---

## Step 9 — Boot the robot at home

Power on the robot. It should:

1. Scan for Wi-Fi networks
2. Find your home SSID (`Wi-Fi Repeater 2` in the example)
3. Connect using the saved password
4. Obtain an IP address from your router (DHCP)

---

## Step 10 — Find the robot IP from your laptop

Scan your home subnet (adjust range if your router uses a different network):

```bash
nmap -sn 192.168.1.0/24
```

Or list ARP entries:

```bash
arp -a
```

Look for the hostname or MAC address of the Raspberry Pi.

---

## Step 11 — SSH into the robot

```bash
ssh deec@192.168.1.xxx
```

Replace `192.168.1.xxx` with the IP from Step 10.

---

## Troubleshooting

| Problem | Action |
|---------|--------|
| No Wi-Fi at home | Check `chmod 600` and `root:root` ownership on `.nmconnection` files |
| Connects to lab only | Confirm home SSID spelling; raise `autoconnect-priority` on home profile |
| No IP / no SSH | Verify `[ipv4] method=auto`; remove static `address1` from home profile |
| Wrong password | Edit `psk=` in `HomeWiFi.nmconnection`, fix permissions, reinsert SD card |

---

## Security note

Do not commit real Wi-Fi passwords to Git. Keep `HomeWiFi.nmconnection` only on the robot SD card or store credentials locally outside the repository.
