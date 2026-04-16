# RTK Base Resources in North America

A comprehensive reference covering free public correction networks, community casters, PPP services, and paid subscription networks. Free and no-registration services are covered in the most detail; paid services include known pricing and access notes.

---

## How RTK Corrections Are Delivered

All services below deliver corrections through one of four mechanisms:

| Mechanism | Description | Internet required |
|---|---|:---:|
| **NTRIP** (Networked Transport of RTCM via IP) | Most common. Rover connects to a caster over TCP port 2101, selects a mountpoint, receives RTCM3 stream. | Yes |
| **SPARTN over IP** | Compact binary format (u-blox PointPerfect). PPP-RTK corrections streamed via MQTT or NTRIP. | Yes |
| **L-band satellite** | Corrections broadcast from geostationary satellites. No cell coverage needed. | No |
| **E6-B direct** | Galileo HAS — receiver decodes corrections from Galileo E6-B signal. No internet, no subscription. | No |

---

## Part 1 — No-Cost Services

---

### 1.1 RTK2GO — Community NTRIP Caster

**Coverage:** Worldwide; density varies by contributor activity. ~800+ active mountpoints; many concentrated in the US.

**What it is:** A free, public NTRIP caster run on the SNIP platform. Anyone can both **publish** a base station stream and **consume** a stream. Corrections quality depends entirely on whoever set up the nearest base — there are no SLAs.

**Access — Rover (consumer)**

1. Browse active mountpoints at `http://monitor.use-snip.com/map` or the table at `http://monitor.use-snip.com/?hostUrl=rtk2go.com&port=2101`
2. Pick the nearest mountpoint by geography.
3. Configure your NTRIP client:
   - **Host:** `rtk2go.com`
   - **Port:** `2101`
   - **Mountpoint:** `<exact name, case-sensitive>`
   - **Username:** your email address
   - **Password:** `none`
4. No registration required for rover use.

**Access — Base (publisher)**

1. Register a free account at `http://rtk2go.com`
2. Configure your base station software (u-center, RTKLIB, str2str, etc.) to push an RTCM3 stream to `rtk2go.com:2101`.
3. Mountpoint name and password are set during registration.

**Correction format:** RTCM3 (format depends on the contributing base; many send MSM4 or MSM7).

**Notes:**
- Stream uptime is not guaranteed — community bases go offline without notice.
- Check the monitor map before depending on a specific mountpoint.
- Excellent for testing and for areas where no official network exists.

---

### 1.2 State DOT / Public Agency Networks (USA)

Approximately 21 states operate RTK networks as a public service. Most require a free registration. Coverage is limited to that state's road network footprint, but station density is usually high (20–50 km inter-station spacing).

#### Free-Access State Networks

| State | Network Name | Software | Registration | NTRIP Host |
|---|---|---|:---:|---|
| **Arkansas** | ARDOT RTN | Pivot | Free account | Contact ARDOT |
| **Florida** | FPRN | SpiderNet | Free account | `fprn.dot.state.fl.us:2101` |
| **Indiana** | INDOT InCORS | Pivot | Free account | `incors.in.gov:2101` |
| **Iowa** | IaRTN | SpiderNet | Free account | Contact Iowa DOT |
| **New York** | NYSNet | SpiderNet | Free account | `cors.dot.ny.gov:2101` |
| **Ohio** | ODOT VRS | Pivot | Free account | Contact ODOT |
| **Oregon** | ORGN | SpiderNet | Free account | Contact ODOT |

**Typical registration process (all state networks):**
1. Visit the state DOT CORS/survey division website.
2. Submit a free account request (name, affiliation, email, intended use).
3. Receive login credentials (usually within 1–3 business days).
4. Configure NTRIP client with provided host, port, username, and password.
5. Select a VRS (Virtual Reference Station) mountpoint — the network generates a synthetic base position near your rover on the fly.

#### Notable Restricted / Paid State Networks

| State | Network | Status | Notes |
|---|---|---|---|
| Texas | TxDOT RTN | Restricted | TxDOT employees and contractors only. Private alternatives required for general use. |
| North Carolina | NC RTN | Paid | Annual subscription; Pivot network. |
| California | CRTN | Varies | Operated by Caltrans; access through affiliated programs. |

> **Finding your state:** E38 Survey Solutions maintains a current state-by-state list at [e38surveysolutions.com/pages/ntrip-rtk-network-access-by-state](https://e38surveysolutions.com/pages/ntrip-rtk-network-access-by-state). ArduSimple also maintains a list at [ardusimple.com/rtk-correction-services-and-ntrip-casters-in-the-united-states-of-america-usa/](https://www.ardusimple.com/rtk-correction-services-and-ntrip-casters-in-the-united-states-of-america-usa/).

---

### 1.3 NOAA Continuously Operating Reference Station (CORS) Network — NCN

**Coverage:** ~2,300+ stations across the US and territories.

**What it is:** A cooperative network of GNSS reference stations managed by NOAA/NGS. Stations are contributed by federal agencies, state DOTs, universities, and municipalities.

**Important limitation:** The NCN is designed for **static post-processing**, not real-time RTK streaming. NGS does not operate a live NTRIP caster from NCN data.

**What you can use it for:**
- Download 1-second or 30-second RINEX observation files for PPK (post-processed kinematic) work — free.
- Identify nearby stations to set up your own short-baseline post-processing.
- Many NCN contributor agencies (state DOTs, universities) do separately stream real-time data; those are listed under state DOT networks above.

**Access:**
1. Visit [geodesy.noaa.gov/CORS](https://geodesy.noaa.gov/CORS/)
2. Use the interactive map to find stations near your work area.
3. Download RINEX files directly — no account required.
4. For real-time data, check whether the owning agency provides an NTRIP feed independently.

---

### 1.4 UNAVCO / EarthScope NOTA Network

**Coverage:** ~1,100+ stations concentrated in the western US, Alaska, Hawaii, and Caribbean. Also South and Central America through the broader NOTA network.

**What it is:** EarthScope Consortium (formerly UNAVCO) operates the Network of the Americas (NOTA), a GPS/GNSS geodetic network primarily for earth science. A subset of stations broadcasts real-time NTRIP streams.

**Free access process:**
1. Register at [earthscope.org](https://www.earthscope.org) — free for educational/research use.
2. Request real-time data access credentials.
3. Connect NTRIP client to the EarthScope caster (credentials provided after registration).
4. Station coverage is sparse in the central and eastern US.

**Suitable for:** PPK, atmospheric research, long-baseline corrections. Not optimized for RTK rover work (inter-station spacing is often 100+ km).

---

### 1.5 Galileo High Accuracy Service (HAS) — Free Global PPP

**Coverage:** Global (wherever Galileo satellites are visible, ~±75° latitude).

**What it is:** A free, encrypted-but-open Precise Point Positioning (PPP) service broadcast directly on the Galileo E6-B signal. No internet, no subscription, no base station required.

**Accuracy and convergence (realistic performance):**

| Metric | Typical value |
|---|---|
| Horizontal steady-state accuracy | ~10–20 cm (95%) |
| Vertical steady-state accuracy | ~20–40 cm (95%) |
| Convergence to 20 cm horizontal | ~15–30 minutes |
| Convergence to 5 cm horizontal | 1.5–4 hours |

> HAS is **not RTK** — it is PPP. It achieves sub-decimeter accuracy but does not achieve the centimeter-level, fast-fix RTK accuracy of a short-baseline correction service.

**ZED-F9P compatibility:**
- HAS decoding requires firmware HPG 1.30+ (ZED-F9P-04B or 05B).
- The ZED-F9P-15B (L1/L5) does not support HAS as of firmware L1L5 1.40.
- Enable via UBX-CFG-NAVSPG, set the GNSS constellation to include Galileo and enable E6 reception.

**How to use:**
1. Ensure receiver is ZED-F9P-04B or 05B with firmware ≥ HPG 1.30.
2. Verify antenna has clear sky view (HAS requires E6-B signal, ~1278.75 MHz — most survey-grade active patch antennas cover this).
3. In u-center: enable Galileo, E6-B signal, and SPARTN/HAS decoding in the configuration view.
4. Allow 15–30 minutes warm-up after first lock before depending on the solution.

**Key trade-off vs RTK:** HAS never requires a data connection and works anywhere with Galileo visibility, but it will never replace RTK for sub-5 cm real-time work. It is most useful as a fallback when cell coverage is unavailable.

---

### 1.6 RTKdata.online — Free NTRIP Aggregator / Map

**What it is:** A web tool that aggregates and maps public NTRIP mountpoints from multiple casters (RTK2GO, state networks, etc.) in one place. Useful for discovery but not itself a correction provider.

**Access:** Visit [rtkdata.online](https://rtkdata.online/) — no account required.

---

### 1.7 GEODNET — 30-Day Free Trial (Blockchain-based Network)

**Coverage:** 5,400+ stations globally; strong North American coverage.

**What it is:** A decentralized, crowd-sourced GNSS reference network where station operators earn cryptocurrency (GEOD token) for hosting a reference station. Corrections are delivered via NTRIP.

**Accuracy:** Survey-grade RTK; claimed <2 cm horizontal under typical conditions.

**Free access:**
- 30-day free NTRIP trial available at [geodnet.com/free](https://geodnet.com/free).
- No credit card required during trial.
- After trial: paid subscription required (see Part 2 below).

**Trial access process:**
1. Visit the free trial page and create an account.
2. Receive NTRIP credentials (host, port, mountpoint, username, password).
3. Configure rover NTRIP client with provided details.

---

### 1.8 Canadian Public Resources

Canada lacks a single unified free real-time RTK network equivalent to US state DOT networks. The primary federal resource is post-processing only.

#### NRCan CSRS-PPP (Post-Processing, Free)

**What it is:** Natural Resources Canada's Canadian Spatial Reference System Precise Point Positioning web service. Upload a RINEX file and receive a PPP-processed position in return.

- Free, no account required for basic use.
- Access at [webapp.csrs-scrs.nrcan-rncan.gc.ca](https://webapp.csrs-scrs.nrcan-rncan.gc.ca)
- Suitable for static survey control points; not real-time.

#### NRCan Real-Time RTK Networks Registry

NRCan maintains a registry of private RTK networks that have signed compliance agreements to tie into NAD83(CSRS). These are mostly paid commercial networks (see Part 2), but the registry is useful for finding what is available in a specific province.

- Registry: [webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/data-donnees/rtk.php](https://webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/data-donnees/rtk.php)

#### Provincial Networks (Mostly Paid)

| Province | Network | Cost |
|---|---|---|
| Alberta | ABCORS (via Trimble VRS Now) | Paid |
| British Columbia | BC CORS | Varies — some provincial access |
| Ontario | Ontario SmartNet / various | Paid |
| Quebec | Various | Paid |

---

## Part 2 — Paid Services

---

### 2.1 Hexagon HxGN SmartNet North America

**Coverage:** Extensive US and Canadian coverage; one of the largest proprietary RTK networks in North America.

**Technology:** Network RTK (VRS — Virtual Reference Station); NTRIP delivery.

**Pricing (approximate):**

| Plan | Cost |
|---|---|
| Single state | ~$2,400/year |
| Multi-state or national | $5,000–$10,000+/year |
| Enterprise / fleet | Contact Hexagon for quote |

**Access process:**
1. Visit [hxgnsmartnet.com](https://hxgnsmartnet.com) and select a subscription plan.
2. Provide billing information and intended use.
3. Receive NTRIP credentials by email.
4. Configure NTRIP client: host typically `smartnetna.com`, port `2101`, with assigned mountpoint.
5. Annual renewal required.

**Best for:** Survey firms, construction, state-level or regional operations requiring guaranteed uptime and support SLAs.

---

### 2.2 Trimble VRS Now

**Coverage:** 1,000,000+ square miles across the US and Canada (following acquisition of MidStates VRS and other regional networks).

**Technology:** Virtual Reference Station (VRS); NTRIP delivery over cellular. Trimble-optimized correction format compatible with non-Trimble rovers via standard RTCM3.

**Accuracy:** 2 cm horizontal / 3 cm vertical (typical).

**Pricing:** Not publicly listed; quote-based. Historically comparable to SmartNet at the state level (~$2,000–$4,000/year per device). Contact Trimble Positioning Services for current rates.

**Access process:**
1. Contact [positioningservices.trimble.com](https://positioningservices.trimble.com/en/vrs) for a quote.
2. Complete subscription agreement.
3. Receive credentials for the VRS Now caster (`vrsnow.us`).
4. Connect NTRIP client with provided host, port, mountpoint, username, password.

**Free trial:** 30-day trial available on request.

---

### 2.3 u-blox PointPerfect (SPARTN / PPP-RTK)

**Coverage:** Continental US and Western Europe (L-band + IP). Expanding globally.

**Technology:** SPARTN protocol delivered via IP (MQTT or NTRIP) or L-band satellite broadcast from Inmarsat. Decoded by ZED-F9P-04B/05B/15B on firmware HPG 1.30+.

**Accuracy:** Claimed ~2 cm horizontal after convergence (PPP-RTK with ambiguity resolution).

**Plans and pricing:**

| Plan | Delivery | Price |
|---|---|---|
| PointPerfect (standard) | IP (MQTT/NTRIP) or L-band | ~$15–$50/device/month (varies by reseller and region) |
| PointPerfect Flex | IP (MQTT) | Usage-based; pay per hour — targeted at seasonal users (farmers, surveyors) |
| PointPerfect Global | IP | Global coverage tier; higher monthly rate |

**Free trial:** 1 month of unlimited access for a single device (IP only).

**Access process:**
1. Create an account on u-blox Thingstream at [portal.thingstream.io](https://portal.thingstream.io)
2. Select the PointPerfect plan and region.
3. Download the current encryption key / authentication token — keys rotate periodically and must be pushed to the receiver.
4. In u-center: configure the SPARTN source (IP or L-band), enter MQTT credentials, enable HPG correction input.
5. For L-band: no internet required after initial key provisioning; the receiver decodes the satellite broadcast passively.

**Notes:**
- Key provisioning requires internet even for L-band operation (keys change approximately monthly).
- SparkFun RTK firmware has built-in PointPerfect management.
- Native to the ZED-F9P-04B and 05B; no additional SPARTN decode hardware needed.

---

### 2.4 GEODNET (Post-Trial)

**Coverage:** 5,400+ global stations; dense in North America, Europe, India.

**Technology:** NTRIP; community-contributed stations, professionally operated caster.

**Pricing (post-trial):**

| Plan | Cost |
|---|---|
| Standard NTRIP access | ~$40/month (via resellers such as ROCK RTK) |
| Annual | ~$400/year |

**Access process (after trial):**
1. Subscribe at [geodnet.com](https://geodnet.com) or through a reseller (ROCK Robotic, etc.).
2. Receive or renew NTRIP credentials.
3. Connect rover NTRIP client as during trial period.

---

### 2.5 Point One Navigation — Polaris RTK

**Coverage:** Continental US; strong coverage in populated areas via NTRIP.

**Technology:** VRS NTRIP. Designed for autonomous systems and robotics as well as survey.

**Pricing:**

| Plan | Cost |
|---|---|
| Survey grade | $150/month or $1,500/year |
| Fleet / enterprise | Contact for volume pricing |

**Free trial:** 14 days.

**Access process:**
1. Sign up at [pointonenav.com](https://pointonenav.com)
2. Receive NTRIP credentials.
3. Configure rover client with Polaris caster details.

---

### 2.6 RTKdata.com

**Coverage:** North America, Europe, and broader global coverage via aggregated streams.

**Technology:** NTRIP (aggregates and re-serves streams from public CORS and contributed bases).

**Pricing:**

| Plan | Cost |
|---|---|
| Single region | $40/month |
| Global | $69/month |

**Access process:**
1. Subscribe at [rtkdata.com](https://rtkdata.com)
2. Receive NTRIP caster credentials.
3. Browse mountpoints and configure rover.

---

### 2.7 ROCK RTK (Built on GEODNET)

**Coverage:** 16,000+ stations globally; US coverage is dense.

**Technology:** NTRIP via GEODNET infrastructure.

**Pricing:**

| Plan | Cost |
|---|---|
| Monthly | $40/month |
| Annual | $400/year |

**Access:** [rockrobotic.com/software/rtk-network](https://www.rockrobotic.com/software/rtk-network) — subscribe and receive NTRIP credentials.

---

## Part 3 — Summary Comparison Table

| Service | Cost | Coverage | Accuracy | Internet req. | RTK type | Registration |
|---|:---:|---|:---:|:---:|---|:---:|
| **RTK2GO** | Free | Worldwide (spot) | Varies | Yes | NTRIP/RTCM3 | No (rover) |
| **State DOT networks** | Free | State-level | RTK ~2 cm | Yes | NTRIP/VRS | Free account |
| **NOAA NCN** | Free | US-wide | PPK only | For download | Post-process only | No |
| **EarthScope/UNAVCO NOTA** | Free | W. US heavy | RTK capable | Yes | NTRIP/RTCM3 | Free account |
| **Galileo HAS** | Free | Global | ~10–20 cm | **No** | PPP (E6-B) | None |
| **GEODNET trial** | Free (30 days) | Global | RTK ~2 cm | Yes | NTRIP | Account |
| **NRCan CSRS-PPP** | Free | Canada | PPK cm | For upload | Post-process only | No |
| **GEODNET (paid)** | $40/mo | Global | RTK ~2 cm | Yes | NTRIP | Account |
| **RTKdata.com** | $40–69/mo | N. America / Global | RTK ~2 cm | Yes | NTRIP | Paid |
| **ROCK RTK** | $40/mo | Global | RTK ~2 cm | Yes | NTRIP | Paid |
| **u-blox PointPerfect IP** | $15–50/mo/device | US + Europe | PPP-RTK ~2 cm | Yes | SPARTN/NTRIP | Thingstream |
| **u-blox PointPerfect L-band** | Included in plan | US + Europe | PPP-RTK ~2 cm | **No (post-key)** | SPARTN/L-band | Thingstream |
| **Point One Polaris** | $150/mo or $1,500/yr | US | RTK ~2 cm | Yes | NTRIP/VRS | Paid |
| **Trimble VRS Now** | Quote (~$2–4K/yr) | US + Canada | RTK 2 cm H | Yes | NTRIP/VRS | Paid |
| **HxGN SmartNet** | $2,400–$10K+/yr | N. America | RTK ~2 cm | Yes | NTRIP/VRS | Paid |

---

## Part 4 — Decision Guide

```
Do you need real-time RTK (< 5 cm)?
├── Yes
│   ├── Is there a free state DOT network in your area?
│   │   ├── Yes → Use the free state DOT NTRIP service (see §1.2)
│   │   └── No
│   │       ├── Is there an RTK2GO mountpoint within ~10 km?
│   │       │   ├── Yes → RTK2GO (free, §1.1) — verify stream uptime first
│   │       │   └── No
│   │       │       ├── Budget < $50/month?
│   │       │       │   ├── Yes → GEODNET or ROCK RTK ($40/mo, §2.4/2.7)
│   │       │       │   └── No
│   │       │       │       ├── Need L-band (no cell)?
│   │       │       │       │   ├── Yes → PointPerfect L-band (§2.3)
│   │       │       │       │   └── No → SmartNet or Trimble VRS Now (§2.1/2.2)
│   │       │       └── Try GEODNET 30-day free trial first (§1.7)
└── No — sub-meter to ~20 cm acceptable?
    ├── No cell available → Galileo HAS (free, §1.5) — 15–30 min convergence
    └── Post-processing acceptable → NOAA NCN RINEX (free, §1.3)
```

---

## Part 5 — NTRIP Client Setup Reference

Most GNSS software and firmware (u-center, RTKLIB, SW Maps, Field Genius, etc.) use the same connection parameters:

| Field | Typical value |
|---|---|
| Protocol | NTRIP v1 or v2 (v2 preferred when available) |
| Host | Network-specific (e.g. `rtk2go.com`) |
| Port | `2101` (standard NTRIP port) |
| Mountpoint | Network-specific (case-sensitive) |
| Username | Network-specific (RTK2GO uses your email) |
| Password | Network-specific (RTK2GO uses `none`) |
| Send GGA | Yes — required for VRS networks to generate a virtual base near you |
| GGA interval | Every 5–10 seconds |

For ZED-F9P specifically: configure NTRIP input on UART2 or USB, enable RTCM3 input protocol, and ensure the rover is in `ROVER` mode (UBX-CFG-TMODE3 = 0).

---

## Sources

- [RTK correction services in the USA — ArduSimple](https://www.ardusimple.com/rtk-correction-services-and-ntrip-casters-in-the-united-states-of-america-usa/)
- [NTRIP/RTK Network Access by State — E38 Survey Solutions](https://e38surveysolutions.com/pages/ntrip-rtk-network-access-by-state)
- [North America NTRIP list — ntrip-list.com](https://ntrip-list.com/north-america/)
- [RTK Correction Providers North America — RTK Directory](https://rtkdirectory.com/rtk-correction-service-providers/north-america/)
- [RTK2GO — Community NTRIP Caster](http://rtk2go.com/)
- [RTK2GO How to Connect](http://rtk2go.com/how-to-connect/)
- [NOAA CORS Network (NCN)](https://geodesy.noaa.gov/CORS/)
- [NOAA NCN FAQs](https://geodesy.noaa.gov/CORS/cors_faqs.shtml)
- [NRCan RTK Networks](https://webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/data-donnees/rtk.php)
- [GEODNET Free RTK Trial](https://geodnet.com/free)
- [GEODNET Coverage Map](https://rtk.geodnet.com/coverage/)
- [Free Public NTRIP Services — Tiptop Surveying](https://tiptopsurveying.com/free-public-ntrip-services-pros-cons-and-how-to-access-them/)
- [Finally, a list of public RTK base stations in the US — GPS World](https://www.gpsworld.com/finally-a-list-of-public-rtk-base-stations-in-the-u-s/)
- [HxGN SmartNet North America](https://hxgnsmartnet.com/)
- [HxGN SmartNet pricing context — Point One Nav review](https://pointonenav.com/news/smartnet-review-hxgn-alternatives/)
- [Trimble VRS Now](https://positioningservices.trimble.com/en/vrs)
- [u-blox PointPerfect Flex](https://www.u-blox.com/en/product/pointperfectflex)
- [PointPerfect getting started — Thingstream](https://developer.thingstream.io/guides/location-services/pointperfect-getting-started)
- [Point One Navigation Polaris — E38 Survey Solutions](https://e38surveysolutions.com/products/point-one-polaris-rtk-ntrip-service-annual-license)
- [ROCK RTK Network](https://www.rockrobotic.com/software/rtk-network/)
- [RTKdata.com pricing](https://rtkdata.com/pricing/)
- [Galileo HAS PPP with ZED-F9P — MDPI Sensors 2023](https://www.mdpi.com/1424-8220/23/13/6074)
- [Affordable Real-Time PPP with Galileo HAS — MDPI Remote Sensing 2024](https://www.mdpi.com/2072-4292/16/21/4008)
- [Correction Sources — SparkFun RTK Product Manual](https://docs.sparkfun.com/SparkFun_RTK_Firmware/correction_sources/)
- [RTK corrections explained — Emlid Blog](https://blog.emlid.com/rtk-corrections-explained-from-base-station-to-ntrip-service/)
