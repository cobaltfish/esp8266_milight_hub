#include "wireguardmanager.h"
#include <Arduino.h>


#include <cinttypes>
#include <ctime>
#include <functional>
#include <thread> 


#include <esp_wireguard.h>
#include <esp_wireguard_err.h>
static const char *const TAG = "wireguard";

/*
 * Cannot use `static const char*` for LOGMSG_PEER_STATUS on esp8266 platform
 * because log messages in `WireGuardManager::update()` method fail.
 */
#define LOGMSG_PEER_STATUS "WireGuard remote peer is %s (latest handshake %s)"

static const char *const LOGMSG_ONLINE = "online";
static const char *const LOGMSG_OFFLINE = "offline";


void WireGuardManager::setup() {
  Serial.println(F("Initializing WireGuard..."));

  if (!this->srctime_) {
    Serial.println(F("Error: srctime_ is null"));
    return;
  }

  this->wg_config_.address = this->address_.c_str();
  this->wg_config_.private_key = this->private_key_.c_str();
  this->wg_config_.endpoint = this->peer_endpoint_.c_str();
  this->wg_config_.public_key = this->peer_public_key_.c_str();
  this->wg_config_.port = this->peer_port_;
  this->wg_config_.netmask = this->netmask_.c_str();
  this->wg_config_.persistent_keepalive = this->keepalive_;

  if (this->preshared_key_.length() > 0)
    this->wg_config_.preshared_key = this->preshared_key_.c_str();

  this->publish_enabled_state();

  this->wg_initialized_ = esp_wireguard_init(&(this->wg_config_), &(this->wg_ctx_));

  if (this->wg_initialized_ == ESP_OK) {
    Serial.println(F("WireGuard initialized"));
    this->wg_peer_offline_time_ = millis();
    this->srctime_->add_on_time_sync_callback(std::bind(&WireGuardManager::start_connection_, this));
    defer(std::bind(&WireGuardManager::start_connection_, this));  // defer to avoid blocking setup

#ifdef USE_TEXT_SENSOR
    if (this->address_sensor_ != nullptr) {
      this->address_sensor_->publish_state(this->address_);
    }
#endif
  } else {
    Serial.printf("Error: cannot initialize WireGuard, error code %d\n", this->wg_initialized_);
  }
}

// Add the defer function
void WireGuardManager::defer(std::function<void()> func, int delay_ms) {
  std::thread([func, delay_ms]() {
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    func();
  }).detach();
}

void WireGuardManager::defer(const std::string &name, std::function<void()> &&func, int delay_ms) {  // NOLINT
  std::thread([func, delay_ms]() {
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    func();
  }).detach();
}

void WireGuardManager::loop() {
  if (!this->enabled_) {
    return;
  }

  if ((this->wg_initialized_ == ESP_OK) && (this->wg_connected_ == ESP_OK) && (!this->is_network_connected())) {
    // ESP_LOGV(TAG, "local network connection has been lost, stopping WireGuard...");
    this->stop_connection_();
  }
}

bool WireGuardManager::is_network_connected() {
  // Implement the logic to check network connection status
  return true; // Placeholder implementation
}

void WireGuardManager::update() {
  bool peer_up = this->is_peer_up();
  time_t lhs = this->get_latest_handshake();
  bool lhs_updated = (lhs > this->latest_saved_handshake_);

  // ESP_LOGV(TAG, "enabled=%d, connected=%d, peer_up=%d, handshake: current=%.0f latest=%.0f updated=%d",
          //  (int) this->enabled_, (int) (this->wg_connected_ == ESP_OK), (int) peer_up, (double) lhs,
          //  (double) this->latest_saved_handshake_, (int) lhs_updated);

  if (lhs_updated) {
    this->latest_saved_handshake_ = lhs;
  }

  std::string latest_handshake =
      (this->latest_saved_handshake_ > 0)
          ? [] (time_t t) {
              char buf[64];
              struct tm *tm_info = localtime(&t);
              strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", tm_info);
              return std::string(buf);
            } (this->latest_saved_handshake_)
          : "timestamp not available";

  if (peer_up) {
    if (this->wg_peer_offline_time_ != 0) {
      ESP_LOGI(TAG, LOGMSG_PEER_STATUS, LOGMSG_ONLINE, latest_handshake.c_str());
      // Serial.printf(LOGMSG_PEER_STATUS, LOGMSG_ONLINE, latest_handshake.c_str());
      this->wg_peer_offline_time_ = 0;
    } else {
      ESP_LOGD(TAG, LOGMSG_PEER_STATUS, LOGMSG_ONLINE, latest_handshake.c_str());
      // Serial.printf(LOGMSG_PEER_STATUS, LOGMSG_ONLINE, latest_handshake.c_str());
    }
  } else {
    if (this->wg_peer_offline_time_ == 0) {
      ESP_LOGW(TAG, LOGMSG_PEER_STATUS, LOGMSG_OFFLINE, latest_handshake.c_str());
      // Serial.printf(LOGMSG_PEER_STATUS, LOGMSG_OFFLINE, latest_handshake.c_str());
      this->wg_peer_offline_time_ = millis();
    } else if (this->enabled_) {
      ESP_LOGD(TAG, LOGMSG_PEER_STATUS, LOGMSG_OFFLINE, latest_handshake.c_str());
      // Serial.printf(LOGMSG_PEER_STATUS, LOGMSG_OFFLINE, latest_handshake.c_str());
      this->start_connection_();
    }

    // check reboot timeout every time the peer is down
    if (this->enabled_ && this->reboot_timeout_ > 0) {
      if (millis() - this->wg_peer_offline_time_ > this->reboot_timeout_) {
        ESP_LOGE(TAG, "WireGuard remote peer is unreachable, rebooting...");
        Serial.println(F("WireGuard remote peer is unreachable, rebooting..."));
        ESP.restart(); // Reboot the device
      }
    }
  }

#ifdef USE_BINARY_SENSOR
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state(peer_up);
  }
#endif

#ifdef USE_SENSOR
  if (this->handshake_sensor_ != nullptr && lhs_updated) {
    this->handshake_sensor_->publish_state((double) this->latest_saved_handshake_);
  }
#endif
}

void WireGuardManager::dump_config() {
  ESP_LOGI(TAG, "WireGuard:");
  ESP_LOGI(TAG, "  Address: %s", this->address_.c_str());
  ESP_LOGI(TAG, "  Netmask: %s", this->netmask_.c_str());
  ESP_LOGI(TAG, "  Private Key: " LOG_SECRET("%s"), mask_key(this->private_key_).c_str());
  ESP_LOGI(TAG, "  Peer Endpoint: " LOG_SECRET("%s"), this->peer_endpoint_.c_str());
  ESP_LOGI(TAG, "  Peer Port: " LOG_SECRET("%d"), this->peer_port_);
  ESP_LOGI(TAG, "  Peer Public Key: " LOG_SECRET("%s"), this->peer_public_key_.c_str());
  ESP_LOGI(TAG, "  Peer Pre-shared Key: " LOG_SECRET("%s"),
                (this->preshared_key_.length() > 0 ? mask_key(this->preshared_key_).c_str() : "NOT IN USE"));
  ESP_LOGI(TAG, "  Peer Allowed IPs:");
  for (auto &allowed_ip : this->allowed_ips_) {
    ESP_LOGI(TAG, "    - %s/%s", std::get<0>(allowed_ip).c_str(), std::get<1>(allowed_ip).c_str());
  }
  ESP_LOGI(TAG, "  Peer Persistent Keepalive: %d%s", this->keepalive_,
                (this->keepalive_ > 0 ? "s" : " (DISABLED)"));
  ESP_LOGI(TAG, "  Reboot Timeout: %" PRIu32 "%s", (this->reboot_timeout_ / 1000),
                (this->reboot_timeout_ != 0 ? "s" : " (DISABLED)"));
  // be careful: if proceed_allowed_ is true, require connection is false
  ESP_LOGI(TAG, "  Require Connection to Proceed: %s", (this->proceed_allowed_ ? "NO" : "YES"));
  // LOG_UPDATE_INTERVAL(this);
  Serial.println(F("WireGuard:"));
  Serial.print(F("  Address: "));
  Serial.println(this->address_.c_str());
  Serial.print(F("  Netmask: "));
  Serial.println(this->netmask_.c_str());
  Serial.print(F("  Private Key: "));
  Serial.println(mask_key(this->private_key_).c_str());
  Serial.print(F("  Peer Endpoint: "));
  Serial.println(this->peer_endpoint_.c_str());
  Serial.print(F("  Peer Port: "));
  Serial.println(this->peer_port_);
  Serial.print(F("  Peer Public Key: "));
  Serial.println(this->peer_public_key_.c_str());
  Serial.print(F("  Peer Pre-shared Key: "));
  Serial.println(this->preshared_key_.length() > 0 ? mask_key(this->preshared_key_).c_str() : "NOT IN USE");
  Serial.println(F("  Peer Allowed IPs:"));
  for (auto &allowed_ip : this->allowed_ips_) {
    Serial.print(F("    - "));
    Serial.print(std::get<0>(allowed_ip).c_str());
    Serial.print(F("/"));
    Serial.println(std::get<1>(allowed_ip).c_str());
  }
  Serial.print(F("  Peer Persistent Keepalive: "));
  Serial.print(this->keepalive_);
  Serial.println(this->keepalive_ > 0 ? "s" : " (DISABLED)");
  Serial.print(F("  Reboot Timeout: "));
  Serial.print(this->reboot_timeout_ / 1000);
  Serial.println(this->reboot_timeout_ != 0 ? "s" : " (DISABLED)");
  Serial.print(F("  Require Connection to Proceed: "));
  Serial.println(this->proceed_allowed_ ? "NO" : "YES");
}
void WireGuardManager::on_shutdown() { this->stop_connection_(); }

bool WireGuardManager::can_proceed() { return (this->proceed_allowed_ || this->is_peer_up() || !this->enabled_); }

bool WireGuardManager::is_peer_up() const {
  return (this->wg_initialized_ == ESP_OK) && (this->wg_connected_ == ESP_OK) &&
         (esp_wireguardif_peer_is_up(&(this->wg_ctx_)) == ESP_OK);
}

time_t WireGuardManager::get_latest_handshake() const {
  time_t result;
  if (esp_wireguard_latest_handshake(&(this->wg_ctx_), &result) != ESP_OK) {
    result = 0;
  }
  return result;
}

void WireGuardManager::set_address(const std::string &address) { this->address_ = address; }
void WireGuardManager::set_netmask(const std::string &netmask) { this->netmask_ = netmask; }
void WireGuardManager::set_private_key(const std::string &key) { this->private_key_ = key; }
void WireGuardManager::set_peer_endpoint(const std::string &endpoint) { this->peer_endpoint_ = endpoint; }
void WireGuardManager::set_peer_public_key(const std::string &key) { this->peer_public_key_ = key; }
void WireGuardManager::set_peer_port(const uint16_t port) { this->peer_port_ = port; }
void WireGuardManager::set_preshared_key(const std::string &key) { this->preshared_key_ = key; }

void WireGuardManager::add_allowed_ip(const std::string &ip, const std::string &netmask) {
  
  ESP_LOGI(TAG, "Adding IP: %s/%s", ip.c_str(), netmask.c_str());
  this->allowed_ips_.emplace_back(ip, netmask);
}

void WireGuardManager::set_keepalive(const uint16_t seconds) { this->keepalive_ = seconds; }
void WireGuardManager::set_reboot_timeout(const uint32_t seconds) { this->reboot_timeout_ = seconds; }
void WireGuardManager::set_srctime(RealTimeClock *srctime) { this->srctime_ = srctime; }



void WireGuardManager::disable_auto_proceed() { this->proceed_allowed_ = false; }

void WireGuardManager::enable() {
  this->enabled_ = true;
  ESP_LOGI(TAG, "WireGuard enabled");
  this->publish_enabled_state();
}

void WireGuardManager::disable() {
  this->enabled_ = false;
  this->defer(std::bind(&WireGuardManager::stop_connection_, this));  // defer to avoid blocking running loop
  ESP_LOGI(TAG, "WireGuard disabled");
  this->publish_enabled_state();
}

void WireGuardManager::publish_enabled_state() {
#ifdef USE_BINARY_SENSOR
  if (this->enabled_sensor_ != nullptr) {
    this->enabled_sensor_->publish_state(this->enabled_);
  }
#endif
}

bool WireGuardManager::is_enabled() { return this->enabled_; }

void WireGuardManager::start_connection_() {
  if (!this->enabled_) {
    ESP_LOGV(TAG, "WireGuard is disabled, cannot start connection");
    return;
  }

  if (this->wg_initialized_ != ESP_OK) {
    ESP_LOGE(TAG, "cannot start WireGuard, initialization in error with code %d", this->wg_initialized_);
    return;
  }

  if (!this->is_network_connected()) {
    ESP_LOGD(TAG, "WireGuard is waiting for local network connection to be available");
    return;
  }

  // if (!this->srctime_->now().is_valid()) {
  //   ESP_LOGD(TAG, "WireGuard is waiting for system time to be synchronized");
  //   return;
  // }

  if (this->wg_connected_ == ESP_OK) {
    ESP_LOGV(TAG, "WireGuard connection already started");
    return;
  }

  ESP_LOGD(TAG, "starting WireGuard connection...");
  this->wg_connected_ = esp_wireguard_connect(&(this->wg_ctx_));

  if (this->wg_connected_ == ESP_OK) {
    ESP_LOGI(TAG, "WireGuard connection started");
  } else if (this->wg_connected_ == ESP_ERR_RETRY) {
    ESP_LOGD(TAG, "WireGuard is waiting for endpoint IP address to be available");
    return;
  } else {
    ESP_LOGW(TAG, "cannot start WireGuard connection, error code %d", this->wg_connected_);
    return;
  }

  ESP_LOGD(TAG, "configuring WireGuard allowed IPs list...");
  bool allowed_ips_ok = true;
  for (const auto& ip : this->allowed_ips_) {
    allowed_ips_ok &= (esp_wireguard_add_allowed_ip(&(this->wg_ctx_), std::get<0>(ip).c_str(), std::get<1>(ip).c_str()) == ESP_OK);
  }

  if (allowed_ips_ok) {
    ESP_LOGD(TAG, "allowed IPs list configured correctly");
  } else {
    ESP_LOGE(TAG, "cannot configure WireGuard allowed IPs list, aborting...");
    for (const auto& ip : this->allowed_ips_) {
      ESP_LOGE(TAG, "Failed IP: %s/%s", std::get<0>(ip).c_str(), std::get<1>(ip).c_str());
    }
    this->stop_connection_();
    // this->mark_failed();
  }
}

void WireGuardManager::stop_connection_() {
  if (this->wg_initialized_ == ESP_OK && this->wg_connected_ == ESP_OK) {
    ESP_LOGD(TAG, "stopping WireGuard connection...");
    esp_wireguard_disconnect(&(this->wg_ctx_));
    this->wg_connected_ = ESP_FAIL;
  }
}

std::string mask_key(const std::string &key) { return (key.substr(0, 5) + "[...]="); }

// WireGuardManager::WireGuardManager() : running(false) {}

// void WireGuardManager::begin(const Settings& settings) {
//   if (settings.wireGuardEnabled) {
//     IPAddress localIP, dns, allowedIPs;
//     localIP.fromString(settings.wgLocalIP.c_str());
//     dns.fromString(settings.wgDNS.c_str());
//     allowedIPs.fromString(settings.wgAllowedIPs.c_str());

//     wg.begin(
//       localIP,
//       settings.wgPrivateKey.c_str(),
//       settings.wgEndpoint.c_str(),
//       settings.wgPublicKey.c_str(),
//       51820
//     );
//     running = true;
//     Serial.println(F("WireGuard VPN started"));
//   } else {
//     Serial.println(F("WireGuard VPN is disabled using default"));
    
//     IPAddress localIP, subnet, gateway, dns, allowedIPs;
//     localIP.fromString("192.168.0.125");
//     gateway.fromString("192.168.0.1");
//     subnet.fromString("255.255.255.0");
//     dns.fromString("8.8.8.8");
//     allowedIPs.fromString("0.0.0.0");
    
// //     bool begin(const IPAddress& localIP, const char* privateKey, const char* remotePeerAddress, const char* remotePeerPublicKey, uint16_t remotePeerPort);

//     wg.begin(
//       localIP,
//       "mJRPNYuOFytgVt7UIPsld0PHl/VYXhi1gMO8JosqNU0=",
//       "192.168.0.80",
//       "CawMTQNqflMcjQQjyZRkB0IwWAAH3QjkbxEazMSAd3Q=",
//       51820
//     );
    
//   }
// }

// void WireGuardManager::stop() {
//   if (running) {
//     wg.end();
//     running = false;
//     Serial.println(F("WireGuard VPN stopped"));
//   }
// }

// bool WireGuardManager::isRunning() const {
//   return running;
// }