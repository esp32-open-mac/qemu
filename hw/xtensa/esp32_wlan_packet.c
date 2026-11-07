/**
 * QEMU WLAN access point emulation
 *
 * Copyright (c) 2008 Clemens Kolbitsch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Modifications:
 *  2008-February-24  Clemens Kolbitsch :
 *                                  New implementation based on ne2000.c
 *  18/1/22 Martin Johnson : Modified for esp32 wifi emulation
 */

#include "qemu/osdep.h"

#include "net/net.h"

#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

// the frame checksum isn't used so just put zero in there.
void insertCRC(mac80211_frame *frame) {
    unsigned long crc;
    unsigned char *fcs = (unsigned char *)frame;
    crc = 0;
    memcpy(fcs+frame->frame_length, &crc, 4);
    frame->frame_length += 4;
}

static void add_byte(mac80211_frame *frame, uint8_t data) {
    frame->data_and_fcs[frame->pos++]=data;
    frame->frame_length=IEEE80211_HEADER_SIZE+frame->pos;
}

static void add_data(mac80211_frame *frame, int len,uint8_t *data) {
    for(int i=0;i<len;i++)
        add_byte(frame,data[i]);
}

static void add_tag(mac80211_frame *frame, int tag, int len, unsigned char bytes[]) {
    add_byte(frame,tag);
    add_byte(frame,len);
    add_data(frame,len,bytes);
}

void Esp32_WLAN_init_ap_frame(Esp32WifiState *s, mac80211_frame *frame) {
    frame->sequence_control.sequence_number = s->inject_sequence_number++;
}

static mac80211_frame *new_frame(unsigned type, unsigned subtype) {
    mac80211_frame *frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = type;
    frame->frame_control.sub_type = subtype;
    frame->frame_control._flags = 0;
    frame->frame_control.from_ds = 0;
    frame->frame_control.to_ds = 0;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;
    frame->pos=0;
    return frame;
}


static const char *g_wpa_passphrase;

void Esp32_WLAN_set_passphrase(const char *pass)
{
    g_wpa_passphrase = pass;
}

/* RSN IE for WPA2-PSK with CCMP: version 1, group CCMP, 1 pairwise CCMP,
 * 1 AKM PSK, capabilities 0. Enough for a supplicant to start the 4-way
 * handshake; the AP side of that handshake is built separately. */
static void add_rsn_ie(mac80211_frame *frame) {
    static const uint8_t rsn[] = {
        0x01, 0x00,                         /* version 1 */
        0x00, 0x0f, 0xac, 0x04,             /* group cipher: CCMP */
        0x01, 0x00,                         /* 1 pairwise cipher */
        0x00, 0x0f, 0xac, 0x04,             /* CCMP */
        0x01, 0x00,                         /* 1 AKM */
        0x00, 0x0f, 0xac, 0x02,             /* PSK */
        0x00, 0x00                          /* capabilities */
    };
    add_tag(frame, 48, sizeof(rsn), (unsigned char *)rsn);
}

static void add_rates(mac80211_frame *frame) {
    add_tag(frame,IEEE80211_BEACON_PARAM_RATES,8,(uint8_t[]){0x8b,0x96,0x82,0x84,0x0c,0x18,0x30,0x60});
    add_tag(frame,IEEE80211_BEACON_PARAM_EXTENDED_RATES,4,(uint8_t[]){0x6c,0x12,0x24,0x48});
}

static void add_ssid(mac80211_frame *frame, const char *ssid) {
    add_tag(frame,IEEE80211_BEACON_PARAM_SSID,strlen(ssid),(uint8_t *)ssid);
}

mac80211_frame *Esp32_WLAN_create_beacon_frame(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_BEACON);
    frame->signal_strength=ap->sigstrength;
    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_REALTIME)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=1|(g_wpa_passphrase ? 0x10 : 0);   /* ESS | privacy */
    frame->pos=12;
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    add_tag(frame,IEEE80211_BEACON_PARAM_TIM,4,(uint8_t[]){4,1,3,0,0});
    if (g_wpa_passphrase) add_rsn_ie(frame);
    return frame;
}

static uint16_t in_cksum(uint16_t *addr, int len) {
    int sum = 0;
    uint16_t answer = 0;
    uint16_t *w = addr;
    int nleft = len;
    while (nleft > 1) {
        sum += *w++;
        nleft -= 2;
    }
    /* mop up an odd byte, if necessary */
    if (nleft == 1) {
        *(uint8_t *)(&answer) = *(uint8_t *) w;
        sum += answer;
    }
    sum = (sum >> 16) + (sum & 0xffff);     /* add hi 16 to low 16 */
    sum += (sum >> 16);             /* add carry */
    answer = ~sum;              /* truncate to 16 bits */
    return (answer);
}

static mac80211_frame *Esp32_WLAN_create_dhcp_frame(int cmd_size, uint8_t dhcp_commands[]) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_DATA,IEEE80211_TYPE_DATA_SUBTYPE_DATA);
    frame->frame_control.to_ds=1;
    add_data(frame,8,(uint8_t[]){ 0xaa, 0xaa ,0x03 ,00 ,00 ,00 ,8 ,00});
    dhcp_request_t req={
        {.version_size=0x45,.ttl=0xff,.protocol=0x11,.dest_ip={0xff,0xff,0xff,0xff}},
        {.src_port_l=0x44,.dest_port_l=0x43},
        {.htype=1,.hlen=6,.xid=0x1d3d00,.chaddr={0x10,0x01,0x00,0xc4,0x0a,0x24},
        .magic_cookie=0x63538263}
    };
    int len=sizeof(req)+cmd_size;
    req.ipheader.len_h=len>>8;
    req.ipheader.len_l=len&0xff;
    len=len-sizeof(ip_header_t);
    req.udpheader.len_h=len>>8;
    req.udpheader.len_l=len&0xff;
    req.ipheader.checksum=in_cksum((void *)&req.ipheader,sizeof(ip_header_t));
    add_data(frame,sizeof(req),(uint8_t *)&req);
    add_data(frame,cmd_size,dhcp_commands);
    return frame;
}

/* Minimal DHCP server: hands out a lease without requiring slirp, so the AP is
 * usable with any netdev backend.
 *
 * msgtype: 2 = OFFER, 5 = ACK.
 */
/* Build a complete Ethernet frame carrying a DHCP OFFER/ACK.
 *
 * Returns the frame length. The caller hands this to Esp32_WLAN_receive(), the
 * same entry point slirp's frames use, so the 802.3 -> 802.11 conversion is
 * identical by construction.
 */
int Esp32_WLAN_build_dhcp_reply(uint8_t *eth, const uint8_t *client_mac,
                                uint32_t xid, uint8_t msgtype) {
    static const uint8_t server_mac[6] = {0x52, 0x55, 0x0a, 0x00, 0x02, 0x02};

    memcpy(eth, BROADCAST, 6);
    memcpy(eth + 6, server_mac, 6);
    eth[12] = 0x08; eth[13] = 0x00;              /* IPv4 */

    uint8_t options[] = {
        53, 1, 0,                 /* message type (filled below) */
        54, 4, 10, 0, 2, 2,       /* server identifier */
        51, 4, 0, 0, 0x0e, 0x10,  /* lease 3600 s */
        1, 4, 255, 255, 255, 0,   /* subnet mask */
        3, 4, 10, 0, 2, 2,        /* router */
        6, 4, 10, 0, 2, 3,        /* dns */
        255
    };
    options[2] = msgtype;

    dhcp_request_t req;
    memset(&req, 0, sizeof(req));
    req.ipheader.version_size = 0x45;
    req.ipheader.ttl = 64;
    req.ipheader.protocol = 0x11;
    req.ipheader.source_ip[0] = 10; req.ipheader.source_ip[1] = 0;
    req.ipheader.source_ip[2] = 2;  req.ipheader.source_ip[3] = 2;
    req.ipheader.dest_ip[0] = 255; req.ipheader.dest_ip[1] = 255;
    req.ipheader.dest_ip[2] = 255; req.ipheader.dest_ip[3] = 255;

    req.udpheader.src_port_h = 0; req.udpheader.src_port_l = 0x43;    /* 67 */
    req.udpheader.dest_port_h = 0; req.udpheader.dest_port_l = 0x44;  /* 68 */

    req.dhcp.opcode = 2;             /* BOOTREPLY */
    req.dhcp.htype = 1;
    req.dhcp.hlen = 6;
    req.dhcp.xid = xid;
    memcpy(req.dhcp.chaddr, client_mac, 6);
    req.dhcp.yiaddr[0] = 10; req.dhcp.yiaddr[1] = 0;
    req.dhcp.yiaddr[2] = 2;  req.dhcp.yiaddr[3] = 15;
    req.dhcp.siaddr[0] = 10; req.dhcp.siaddr[1] = 0;
    req.dhcp.siaddr[2] = 2;  req.dhcp.siaddr[3] = 2;
    req.dhcp.magic_cookie = 0x63538263;

    int udp_len = sizeof(udp_header_t) + sizeof(dhcp_t) + sizeof(options);
    int ip_len = sizeof(ip_header_t) + udp_len;
    req.ipheader.len_h = ip_len >> 8;   req.ipheader.len_l = ip_len & 0xff;
    req.udpheader.len_h = udp_len >> 8; req.udpheader.len_l = udp_len & 0xff;
    req.ipheader.checksum = in_cksum((void *)&req.ipheader, sizeof(ip_header_t));

    memcpy(eth + 14, &req, sizeof(req));
    memcpy(eth + 14 + sizeof(req), options, sizeof(options));
    return 14 + ip_len;
}

mac80211_frame *Esp32_WLAN_create_dhcp_request(uint8_t *ip) {
    uint8_t dhcp_commands[]={
        0x35, 1, 3,
        0x39, 2 ,5 ,0xdc ,
        0x32, 4, ip[0],ip[1],ip[2],ip[3],
        0x3d, 0x07, 0x01, 0x3c, 0x61, 0x05, 0x0d, 0x99, 0x24,
        0x37, 0x04, 0x01, 0x03, 0x1c, 0x06,
        0xff, 0, 0
    };
    return Esp32_WLAN_create_dhcp_frame(sizeof(dhcp_commands),dhcp_commands);
}

mac80211_frame *Esp32_WLAN_create_dhcp_discover(void) {
    uint8_t dhcp_commands[]={
        0x35, 1, 1,
        0x39, 2 ,5 ,0xdc ,
        0x0c ,0x09 ,0x65 ,0x73 ,0x70 ,0x72 ,0x65 ,0x73 ,0x73 ,0x69 ,0x66 ,
        0x3d ,0x07, 0x01 ,0x3c ,0x61 ,0x05 ,0x0d ,0x99 ,0x24 ,
        0x37 ,0x04 ,0x01 ,0x03 ,0x1c ,0x06 ,
        0xff, 0,0
    };
    return Esp32_WLAN_create_dhcp_frame(sizeof(dhcp_commands),dhcp_commands);
}

mac80211_frame *Esp32_WLAN_create_association_request(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ);
    add_data(frame,4,(uint8_t []){0x21,4,3,0});
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_ack(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_CTL,IEEE80211_TYPE_CTL_SUBTYPE_ACK);
    frame->frame_length=10;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_probe_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP);
    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_REALTIME)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=1|(g_wpa_passphrase ? 0x10 : 0);
    frame->pos=12;
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    if (g_wpa_passphrase) add_rsn_ie(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_probe_request(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ);
    memcpy(frame->receiver_address,BROADCAST,6);
    memcpy(frame->address_3,BROADCAST,6);
    add_ssid(frame,ap->ssid);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    add_rates(frame);
    return frame;
}


/* EAPOL-Key msg1 (AP -> STA): ANonce, no MIC, ACK set. This is what makes the
 * supplicant start the 4-way handshake; without it the station associates and
 * then times out with reason 204.
 *
 * Layout after the LLC/SNAP header: EAPOL header (ver, type, length) then the
 * key descriptor body. Key info 0x008a = version 2, pairwise, ACK. */

/* ---- WPA2-PSK key derivation (AP side) ---------------------------------- */

#include <gcrypt.h>

static int gcry_ready;

static void wpa_crypto_init(void)
{
    if (!gcry_ready) {
        gcry_check_version(NULL);
        gcry_ready = 1;
    }
}

/* PMK = PBKDF2(HMAC-SHA1, passphrase, SSID, 4096, 32) */
static void wpa_pmk(const char *pass, const char *ssid, uint8_t out[32])
{
    wpa_crypto_init();
    gcry_kdf_derive(pass, strlen(pass), GCRY_KDF_PBKDF2, GCRY_MD_SHA1,
                    ssid, strlen(ssid), 4096, 32, out);
}

/* PRF-X from 802.11i: HMAC-SHA1 over label||0x00||data||counter */
static void wpa_prf(const uint8_t *key, int keylen, const char *label,
                    const uint8_t *data, int datalen, uint8_t *out, int outlen)
{
    int done = 0;
    uint8_t counter = 0;
    wpa_crypto_init();
    while (done < outlen) {
        gcry_md_hd_t h;
        gcry_md_open(&h, GCRY_MD_SHA1, GCRY_MD_FLAG_HMAC);
        gcry_md_setkey(h, key, keylen);
        gcry_md_write(h, label, strlen(label) + 1);
        gcry_md_write(h, data, datalen);
        gcry_md_write(h, &counter, 1);
        const uint8_t *d = gcry_md_read(h, GCRY_MD_SHA1);
        int n = outlen - done < 20 ? outlen - done : 20;
        memcpy(out + done, d, n);
        done += n;
        counter++;
        gcry_md_close(h);
    }
}

static void wpa_ptk(const uint8_t pmk[32], const uint8_t *aa, const uint8_t *spa,
                    const uint8_t *anonce, const uint8_t *snonce, uint8_t ptk[48])
{
    uint8_t data[76];
    /* Min/Max over the *bytes*, not over the pointers (that was a bug). */
    int mac_swap = memcmp(aa, spa, 6) > 0;
    int nonce_swap = memcmp(anonce, snonce, 32) > 0;
    memcpy(data + 0,  mac_swap ? spa : aa, 6);
    memcpy(data + 6,  mac_swap ? aa : spa, 6);
    memcpy(data + 12, nonce_swap ? snonce : anonce, 32);
    memcpy(data + 44, nonce_swap ? anonce : snonce, 32);
    wpa_prf(pmk, 32, "Pairwise key expansion", data, sizeof(data), ptk, 48);
}

static void wpa_mic(const uint8_t *kck, const uint8_t *data, int len, uint8_t out[16])
{
    gcry_md_hd_t h;
    wpa_crypto_init();
    gcry_md_open(&h, GCRY_MD_SHA1, GCRY_MD_FLAG_HMAC);
    gcry_md_setkey(h, kck, 16);
    gcry_md_write(h, data, len);
    memcpy(out, gcry_md_read(h, GCRY_MD_SHA1), 16);
    gcry_md_close(h);
}


/* AES Key Wrap (RFC 3394) over 8-byte blocks, built from AES-ECB because this
 * libgcrypt build has no GCRY_CIPHER_MODE_WRAP. inlen must be a multiple of 8
 * and at least 16; output is inlen + 8 bytes. */
static void aes_key_wrap(const uint8_t kek[16], const uint8_t *in, int inlen, uint8_t *out)
{
    int n = inlen / 8;
    uint8_t a[8] = {0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6};
    uint8_t r[64];
    memcpy(r, in, inlen);

    gcry_cipher_hd_t ch;
    gcry_cipher_open(&ch, GCRY_CIPHER_AES, GCRY_CIPHER_MODE_ECB, 0);
    gcry_cipher_setkey(ch, kek, 16);

    for (int j = 0; j < 6; j++) {
        for (int i = 0; i < n; i++) {
            uint8_t b[16];
            memcpy(b, a, 8);
            memcpy(b + 8, r + 8 * i, 8);
            gcry_cipher_encrypt(ch, b, 16, NULL, 0);
            uint64_t t = (uint64_t)(n * j + i + 1);
            for (int k = 0; k < 8; k++) a[k] = b[k];
            for (int k = 0; k < 8; k++) {
                a[7 - k] ^= (uint8_t)(t >> (8 * k));
            }
            memcpy(r + 8 * i, b + 8, 8);
        }
    }
    gcry_cipher_close(ch);
    memcpy(out, a, 8);
    memcpy(out + 8, r, inlen);
}

mac80211_frame *Esp32_WLAN_create_eapol_msg1(const uint8_t *sta_mac,
                                             const uint8_t *anonce,
                                             uint64_t replay) {
    /* QoS data (subtype 8): many drivers only dispatch EAPOL from the QoS
     * path. Two bytes of QoS control precede the LLC header. */
    mac80211_frame *frame = new_frame(IEEE80211_TYPE_DATA, 0x08);
    frame->frame_control.to_ds = 0;
    frame->frame_control.from_ds = 1;
    add_data(frame, 2, (uint8_t[]){0x00, 0x00});   /* QoS control: TID 0 */
    add_data(frame, 8, (uint8_t[]){0xaa, 0xaa, 0x03, 0, 0, 0, 0x88, 0x8e});

        uint8_t body[4 + 95];          /* EAPOL header + key descriptor */
    memset(body, 0, sizeof(body));
    body[0] = 0x02;                 /* EAPOL version */
    body[1] = 0x03;                 /* type: key */
    body[2] = 0x00; body[3] = 0x5f;   /* EAPOL length */ /* length 95 */
    body[4] = 0x02;                 /* descriptor type: RSN */
    /* EAPOL-Key fields are BIG-endian: key info 0x008a = version 2 | pairwise
     * | ACK. Writing it little-endian made the supplicant see version 0 and
     * bail with "unsupported EAPOL-Key descriptor version". */
    body[5] = 0x00; body[6] = 0x8a;
    body[7] = 0x00; body[8] = 0x10; /* key length 16, BE */
    for (int i = 0; i < 8; i++) body[9 + i] = (uint8_t)(replay >> (8 * (7 - i)));
    memcpy(body + 17, anonce, 32);  /* ANonce */
    /* IV(16) RSC(8) ID(8) MIC(16) keydatalen(2) left zero */

    add_data(frame, sizeof(body), body);
    (void)sta_mac;
    return frame;
}


/* Derive the PTK the authenticator side needs for the 4-way handshake. */
void Esp32_WLAN_wpa_derive(const char *ssid, const uint8_t *aa, const uint8_t *spa,
                           const uint8_t *anonce, const uint8_t *snonce,
                           uint8_t ptk[48])
{
    uint8_t pmk[32];
    if (!g_wpa_passphrase) { memset(ptk, 0, 48); return; }
    wpa_pmk(g_wpa_passphrase, ssid, pmk);
    wpa_ptk(pmk, aa, spa, anonce, snonce, ptk);
}

/* EAPOL-Key msg3 (AP -> STA): pairwise, install, ACK, MIC, secure. The key
 * data carries the GTK wrapped with the KEK (RFC 3394). */
mac80211_frame *Esp32_WLAN_create_eapol_msg3(const uint8_t *anonce,
                                             const uint64_t replay,
                                             const uint8_t ptk[48]) {
    mac80211_frame *frame = new_frame(IEEE80211_TYPE_DATA, 0x08);
    frame->frame_control.to_ds = 0;
    frame->frame_control.from_ds = 1;
    add_data(frame, 2, (uint8_t[]){0x00, 0x00});
    add_data(frame, 8, (uint8_t[]){0xaa, 0xaa, 0x03, 0, 0, 0, 0x88, 0x8e});

    /* Key data: Group Key KDE (0xDD, OUI 00:0f:ac, type 1) carrying a GTK,
     * AES-KeyWrap'd with the KEK (PTK[16:32]) as 802.11i requires. */
    uint8_t gtk_kde[24];
    memset(gtk_kde, 0, sizeof(gtk_kde));
    gtk_kde[0] = 0xdd; gtk_kde[1] = 22;
    gtk_kde[2] = 0x00; gtk_kde[3] = 0x0f; gtk_kde[4] = 0xac; gtk_kde[5] = 0x01;
    gtk_kde[6] = 0x01;                      /* key id 1, no Tx */
    for (int i = 0; i < 16; i++) gtk_kde[8 + i] = 0x11;

    uint8_t wrapped[32];
    aes_key_wrap(ptk + 16, gtk_kde, sizeof(gtk_kde), wrapped);   /* KEK = PTK[16:32] */

    uint8_t body[4 + 95 + sizeof(wrapped)];
    memset(body, 0, sizeof(body));
    body[0] = 0x02;
    body[1] = 0x03;
    body[2] = 0x00; body[3] = (uint8_t)(95 + sizeof(wrapped));   /* EAPOL length */
    body[4] = 0x02;                       /* RSN descriptor */
    body[5] = 0x13; body[6] = 0xca;       /* + ENCRYPTED_KEY_DATA */
    body[7] = 0x00; body[8] = 0x10;
    for (int i = 0; i < 8; i++) body[9 + i] = (uint8_t)(replay >> (8 * (7 - i)));
    memcpy(body + 17, anonce, 32);
    body[97] = 0x00; body[98] = (uint8_t)sizeof(wrapped);   /* key data length, BE */
    memcpy(body + 99, wrapped, sizeof(wrapped));
    /* MIC over the frame with the key data already encrypted and the MIC zero */
    wpa_mic(ptk, body, sizeof(body), body + 81);                /* KCK = PTK[0:16] */

    add_data(frame, sizeof(body), body);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_authentication_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION);
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Authentication Algorithm (here: Open System)
     *  - Authentication SEQ
     *  - Status code (successful 0x0)
     */
    /* Fixed params only: algorithm 0 (open), seq 2, status 0. The SSID element
     * that used to follow is not part of an authentication response; the open
     * stack tolerated it, the closed one times out. */
    add_data(frame,6,(uint8_t []){0,0,2,0,0,0});
    add_ssid(frame,ap->ssid);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_authentication_request(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION);
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Authentication Algorithm (here: Open System)
     *  - Authentication SEQ
     *  - Status code (successful 0x0)
     */
    //frame->duration_id=0x13a;
    add_data(frame,6,(uint8_t []){0,0,1,0,0,0});
    //add_data(frame,11,(uint8_t []){0,0,0,0,0,0,0,0,0,0,0});
    //add_ssid(frame,ap->ssid);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_deauthentication(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION);
    /*
     * Insert reason code:
     *  "Deauthentication because sending STA is leaving"
     */
    add_data(frame,2,(uint8_t []){3,0});
    return frame;
}

mac80211_frame *Esp32_WLAN_create_association_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP);
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Capability Information
     *  - Status code (successful 0x0)
     *  - Association ID
    */
    add_data(frame,6,(uint8_t []){33,4,0,0,1,0xc0});
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_disassociation(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION);
    /*
     * Insert reason code:
     *  "Disassociation because sending STA is leaving"
     */
    add_data(frame,2,(uint8_t []){3,0});
    return frame;
}

mac80211_frame *Esp32_WLAN_create_data_packet(Esp32WifiState *s, const uint8_t *buf, int size) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_DATA,IEEE80211_TYPE_DATA_SUBTYPE_DATA);

    frame->duration_id = 44;
    /* LLC */
    add_data(frame,6,(uint8_t[]){ 0xaa, 0xaa ,0x03 ,0 ,0 ,0});
    memcpy(frame->data_and_fcs+6, buf+12, size-12);
    frame->frame_length = IEEE80211_HEADER_SIZE + size-6;
    return frame;
}
