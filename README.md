<!-- ============================================================
     DRONE ACHARYAA · Team MUYA CHAKHWI · SIH 2026 · PS 26177
     Single-file README: paste directly into your repo's README.md.
     No local images or asset folders are needed.
     ============================================================ -->

<div align="center">

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=260&section=header&text=MUYA%20CHAKHWI&fontSize=72&fontColor=ffffff&animation=fadeIn&fontAlignY=36&desc=Smart%20India%20Hackathon%202026&descAlignY=58&descSize=22" alt="MUYA CHAKHWI" width="100%"/>

<a href="https://git.io/typing-svg">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=800&size=32&pause=1000&color=F97316&center=true&vCenter=true&width=820&lines=%F0%9F%9A%81+DRONE+ACHARYAA;Intelligence+That+Guides%2C+Eyes+That+Save;AI-Powered+Autonomous+Search-and-Rescue+Drone;Offline-First.+GPS-Denied+Ready.+Human-Verified." alt="Typing animation" />
</a>

<br/>

![SIH 2026](https://img.shields.io/badge/Smart%20India%20Hackathon-2026-0B5CA8?style=for-the-badge)
![Problem Statement](https://img.shields.io/badge/Problem%20Statement-26177-F97316?style=for-the-badge)
![Team ID](https://img.shields.io/badge/Team%20ID-181771-16A34A?style=for-the-badge)
![Theme](https://img.shields.io/badge/Theme-Robotics%20%26%20Drones-9333EA?style=for-the-badge)
![Category](https://img.shields.io/badge/Category-Hardware-DB2777?style=for-the-badge)

![False positives](https://img.shields.io/badge/False%20Positive%20Target-%3C5%25-4ADE80?style=flat-square&labelColor=0B2545)
![Overlap](https://img.shields.io/badge/Survey%20Overlap-30%25-38BDF8?style=flat-square&labelColor=0B2545)
![Bands](https://img.shields.io/badge/Priority%20Bands-4-F97316?style=flat-square&labelColor=0B2545)
![Fix](https://img.shields.io/badge/RTK--free%20Fix-%C2%B12.5%20m-FACC15?style=flat-square&labelColor=0B2545)
![Cloud](https://img.shields.io/badge/Cloud%20Dependency-0-F472B6?style=flat-square&labelColor=0B2545)

<br/>

**🚁 Detect &nbsp;→&nbsp; 📍 Locate &nbsp;→&nbsp; ⚖️ Prioritize &nbsp;→&nbsp; 🧭 Guide &nbsp;·&nbsp; all without the internet**

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 📌 Table of Contents

- [Problem Statement](#-problem-statement)
- [Our Solution: Drone Acharyaa](#-our-solution-drone-acharyaa)
- [How Drone Acharyaa Operates](#-how-drone-acharyaa-operates)
- [What Makes It Different](#-what-makes-it-different)
- [System Architecture](#-system-architecture)
- [Flight Modes](#-flight-modes)
- [Methodologies](#-methodologies)
- [Technology Stack and Components](#-technology-stack-and-components)
- [Technical Workflow During Rescue](#-technical-workflow-during-rescue)
- [Feasibility and Viability](#-feasibility-and-viability)
- [Risks and Solutions](#-risks-and-solutions)
- [Impact and Benefits](#-impact-and-benefits)
- [Business Model](#-business-model)
- [Future Scope](#-future-scope)
- [References](#-references)
- [Meet the Team](#-meet-the-team)

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🎯 Problem Statement

> **PS ID 26177:** A deployable AI-powered autonomous drone that aids search-and-rescue operations by detecting people and hazards, thereby improving responder safety and reducing victim discovery time.

| Field | Details |
|---|---|
| **Theme** | Robotics and Drones |
| **PS Category** | Hardware |
| **Team ID** | 181771 |
| **Team Name** | Muya Chakhwi |

### The problems we address

| # | Problem | Why it matters |
|---|---|---|
| 1 | **Delayed rescue** due to inaccessible or damaged terrain | Every minute lost reduces survival chances |
| 2 | **Connectivity loss** makes cloud-dependent solutions ineffective | Disaster zones often have no cellular or internet |
| 3 | **Risk to rescue teams** during manual assessments | Responders enter unassessed danger zones blindly |
| 4 | **GPS-denied zones** make conventional drone navigation unreliable | Standard drones drift or fail without GPS |
| 5 | **No prioritization among multiple survivors** | Critical cases may be treated as less urgent |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🚁 Our Solution: Drone Acharyaa

**Drone Acharyaa** is a hexacopter-based, edge-AI search-and-rescue platform. It detects survivors and hazards in real time using RGB and thermal vision, locates them, ranks them by urgency, and guides rescuers along safe routes. It does all of this **without depending on the internet**.

### Proposed solution impact

- ✅ Reduces dependency on large human search teams for the initial sweep
- ✅ Keeps data flowing to command centers even with zero cellular or internet infrastructure
- ✅ Removes the need for humans to enter unassessed danger zones blindly
- ✅ Reduces preventable deaths caused by resource misallocation
- ✅ Reduces dependence on imported commercial drone platforms

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🔄 How Drone Acharyaa Operates

```mermaid
flowchart LR
    A(["🛰️ Sense"]) --> B(["🔍 Detect"]) --> C(["⚖️ Prioritize"]) --> D(["🗺️ Map"])
    D --> E(["🧭 Plan"]) --> F(["👷 Guide"]) --> G(["🚨 Alert"])
    G -. continuous loop .-> A

    style A fill:#7FB8E8,stroke:#000,color:#000
    style B fill:#9BE05A,stroke:#000,color:#000
    style C fill:#FFA24D,stroke:#000,color:#000
    style D fill:#FFE06B,stroke:#000,color:#000
    style E fill:#7FB8E8,stroke:#000,color:#000
    style F fill:#9BE05A,stroke:#000,color:#000
    style G fill:#FFA24D,stroke:#000,color:#000
```

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## ⭐ What Makes It Different

| Feature | Description |
|---|---|
| 🐝 **Swarm-based collaborative search** | Multiple drones cover the area together |
| 📴 **Offline AI model integration** | Reliable detection with no cloud dependency |
| 💰 **Cost effective** | Lower operating cost than current SAR drone technologies |
| 🌡️ **RGB + Thermal + GPS + IMU** | Multi-sensor fusion for higher reliability |
| 🛣️ **Dynamic safe corridors** | Constantly updated safe routes for rescuers |
| 🔋 **Energy-aware autonomy** | Optimizes search routes while keeping battery for a safe return |
| 🌊🔥 **Disaster-specific AI** | Built for floods, fire hazards, earthquakes, debris and more |

### 📴 Offline-first, in one picture

```mermaid
flowchart LR
    D["🚁 Drone Acharyaa<br/>edge AI + local SQLite buffer"]
    C["☁️ Cloud / Internet<br/>❌ unreachable"]
    L["📡 LoRa link<br/>compact geo-tagged critical alerts"]
    W["📶 Wi-Fi / 4G<br/>video + images, when available"]
    K["🖥️ Command Center<br/>NDRF / SDRF<br/>live map + ranked queue"]

    D == "always works" ==> L ==> K
    D -. "auto-sync on reconnect" .-> W -.-> K
    D --x C

    style D fill:#7FB8E8,stroke:#000,color:#000
    style C fill:#fecaca,stroke:#ef4444,color:#000
    style L fill:#FFA24D,stroke:#000,color:#000
    style W fill:#FFE06B,stroke:#000,color:#000
    style K fill:#9BE05A,stroke:#000,color:#000
```

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🧩 System Architecture

```mermaid
flowchart LR
    S1["1. SENSE<br/>RGB camera (3-axis)<br/>Thermal / IR camera<br/>LiDAR + Rangefinder<br/>GNSS + IMU + Baro"]
    S2["2. FLY<br/>Autonomous waypoint grid<br/>Geofence + return-to-home<br/>Obstacle stop + avoidance<br/>GPS-denied failover"]
    S3["3. DETECT<br/>YOLO11 on RGB + thermal<br/>Survivor detection<br/>Fire / flood / debris<br/>Thermal confirmation"]
    S4["4. LOCATE<br/>Pixel GPS coordinate<br/>Duplicate survivor merge<br/>Hazard proximity check<br/>Priority score, 4 bands"]
    S5["5. RELAY<br/>LoRa: critical alerts<br/>Wi-Fi / 4G: video + images<br/>Local SQLite buffer<br/>Auto-sync on reconnect"]
    S6["6. COMMAND<br/>Live map + survivor pins<br/>Hazard zone overlay<br/>Safe rescue route + ETA<br/>Ranked priority queue"]

    S1 --> S2 --> S3 --> S4 --> S5 --> S6
    S6 -. feedback .-> S2

    style S1 fill:#7FB8E8,stroke:#000,color:#000
    style S2 fill:#FFE06B,stroke:#000,color:#000
    style S3 fill:#9BE05A,stroke:#000,color:#000
    style S4 fill:#FFA24D,stroke:#000,color:#000
    style S5 fill:#7FB8E8,stroke:#000,color:#000
    style S6 fill:#9BE05A,stroke:#000,color:#000
```

| Stage | Hardware / Software |
|---|---|
| **Sense** | Hexacopter with gimbaled payload |
| **Fly** | Pixhawk 6X v2 flight controller |
| **Detect** | Raspberry Pi 5 running YOLO11 |
| **Locate** | Geo-tagging and triage |
| **Relay** | Dual link, offline first |
| **Command** | Command center (NDRF / SDRF) |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🛫 Flight Modes

| 📍 GPS Available | 📴 Non-GPS / Offline | 🌲 Obstacle Detection |
|---|---|---|
| Waypoint survey grid | Optical flow + rangefinder | Forward rangefinder stop |
| Geofence + altitude cap | EKF source switching | Clearance-altitude survey |
| Coverage map, 30% overlap | Last-known-position hold | Geofence as primary guard |
| RTK-free ±2.5 m fix | Auto-RTH / safe land | Pilot override always live |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🔬 Methodologies

<table>
<tr>
<td width="50%" valign="top">

### 1️⃣ Edge AI Detection
- YOLO11 on RGB + thermal, on-board Raspberry Pi 5
- Detects survivors, fire, flood and debris offline
- Fine-tuned on aerial SAR data (e.g. HERIDAL, HIT-UAV)

</td>
<td width="50%" valign="top">

### 2️⃣ RGB + Thermal Fusion
- Thermal confirmation of every RGB detection
- Filters fire and hot debris, with a target of **<5% false positives**
- Confidence and evidence shown; a human verifies every alert

</td>
</tr>
<tr>
<td width="50%" valign="top">

### 3️⃣ Locate and Prioritize
- Pixel GPS coordinates and duplicate survivor merge
- Hazard proximity check on every detection
- Priority score in 4 bands, giving a ranked rescue queue

</td>
<td width="50%" valign="top">

### 4️⃣ Autonomous Navigation
- Waypoint grid, 30% overlap, geofence, energy-aware return-to-home
- GPS-denied: optical flow with EKF source switching
- Dynamic safe corridors and ETA for rescue teams

</td>
</tr>
</table>

**Comms:** LoRa alerts + SQLite auto-sync &nbsp;|&nbsp; **Validation:** simulation → field trials on recall, false positives and latency

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🛠️ Technology Stack and Components

<div align="center">

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![FastAPI](https://img.shields.io/badge/FastAPI-009688?style=for-the-badge&logo=fastapi&logoColor=white)
![MAVLink](https://img.shields.io/badge/MAVLink-E4572E?style=for-the-badge)
![Leaflet](https://img.shields.io/badge/Leaflet-199900?style=for-the-badge&logo=leaflet&logoColor=white)
![SQLite](https://img.shields.io/badge/SQLite-003B57?style=for-the-badge&logo=sqlite&logoColor=white)
![YOLO11](https://img.shields.io/badge/YOLO11-00A8CC?style=for-the-badge)
![Raspberry Pi 5](https://img.shields.io/badge/Raspberry%20Pi%205-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)
![Pixhawk](https://img.shields.io/badge/Pixhawk-6X%20v2-1F2937?style=for-the-badge)

</div>

| Category | Items |
|---|---|
| **Airframe** | Hexacopter with 3-axis gimbaled payload |
| **Flight controller** | Pixhawk 6X v2 |
| **Onboard compute** | Raspberry Pi 5 |
| **Sensors** | RGB camera, thermal / IR camera, LiDAR + rangefinder, GNSS + IMU + barometer |
| **Communication** | LoRa (critical alerts), Wi-Fi / 4G (video + images) |
| **Software** | Python, FastAPI, MAVLink, Leaflet (live map dashboard), SQLite |
| **Ground control** | DRISHYA V1 GCS (Disaster Response Intelligence & Mission Control System): tactical map, mission planner, live camera, 3D LiDAR, Pi 5 status, survivor detection panel, mission log |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🌊 Technical Workflow During Rescue

```mermaid
sequenceDiagram
    autonumber
    participant R as 👷 Rescuer (ground)
    participant D as 🚁 Drone Acharyaa
    participant P as 🧠 Pi 5 (edge AI)
    participant C as 🖥️ Command Dashboard

    R->>D: Start SAR operation
    D->>D: Detect victims using RGB + thermal camera
    D->>P: Detected images (operating over RF)
    P->>P: Victim attribute identification
    P->>P: Data analysis + priority ranking
    P->>P: Geotagging + route optimization
    P->>C: Store and transmit via SQLite
    C->>R: Live video dashboard with victim ranking
```

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## ✅ Feasibility and Viability

<table>
<tr>
<th width="33%">🔧 Technical</th>
<th width="33%">💰 Economical</th>
<th width="33%">🚚 Deployment and Logistics</th>
</tr>
<tr>
<td valign="top">

- **Embedded edge-AI architecture:** optimized, quantized computer-vision models for real-time victim detection on the edge
- **Multi-sensor fusion pipeline:** hardware-synchronized RGB + LWIR thermal feeds
- **GPS-denied visual odometry:** VIO and optical flow keep positioning, altitude hold and obstacle avoidance working

</td>
<td valign="top">

- **Lower cost per flight hour** than existing SAR drones
- **Rapid capital recovery:** low initial hardware and setup costs
- **Reduced human liability:** replaces high-risk pilot operations in hazardous environments

</td>
<td valign="top">

- **Offline and autonomous:** AI detection continues in network-disrupted zones
- **Low operator dependency:** one operator can supervise multiple drones
- **Rapid and scalable:** deploy and scale by disaster area and severity

</td>
</tr>
</table>

**Cost per hour (search-and-rescue missions):**

| | Existing SAR drones | Drone Acharyaa |
|---|---|---|
| **Operating cost / hour** | ₹5,000 – ₹15,000 | ₹4,000 – ₹8,000 |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## ⚠️ Risks and Solutions

| Risk | Solution | Strategy |
|---|---|---|
| 🔥 Fire, hot debris and heated surfaces may trigger **false survivor detections** | Fuse RGB confirmation, thermal signature analysis and a confidence threshold | Test with survivor, fire, debris and heated-surface scenarios; target **<5% false positives** |
| 📡 **LoRa bandwidth** is insufficient for live RGB / thermal video | Send compact geo-tagged alerts over LoRa; buffer footage onboard | Prioritize high-confidence alerts; sync footage over 5G / Wi-Fi when available |
| 🤖 AI may generate **incorrect or uncertain alerts**, causing hesitation | Show priority ranking, confidence score and RGB / thermal evidence for every detection | **Human verification** before final rescue prioritization, keeping the responder in control |

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🌍 Impact and Benefits

<details open>
<summary><b>Click to expand or collapse the four impact areas</b></summary>

<br/>

| 💼 Economical | 💡 Innovational | 🇮🇳 National | 🤝 Social |
|---|---|---|---|
| Lower search cost | Edge RGB + thermal AI | Offline-first response | Faster survivor identification |
| Smarter responder deployment | Human-verified AI alerts | Scalable multi-drone search | Inaccessible-area search |
| Efficient flight usage | Closed-loop search | Real-time rescue intelligence | Reduced responder exposure |
| Reduced manpower burden | Communication-efficient intelligence | Rapid disaster assessment | Night-time search support |
| Scalable deployment | Modular AI architecture | Multi-agency coordination | Remote-area assistance |
| Reduced search redundancy | Adaptive search intelligence | Connectivity-resilient operations | Rapid disaster assistance |
| Better asset utilization | Multi-modal evidence fusion | Faster resource mobilization | Improved rescue accessibility |

</details>

### Key benefits

1. **Faster and more targeted rescue:** AI survivor detection and prioritization for targeted search and rescue.
2. **Safer rescue operations:** autonomous aerial reconnaissance assesses dangerous, unstable and inaccessible areas before humans enter.
3. **Rescue intelligence without internet dependency:** offline edge AI handles local detection, localization and decision support.
4. **Reliable, evidence-based alerts:** every alert carries AI confidence, GPS location and RGB / thermal evidence, with human verification before critical decisions.

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 💼 Business Model

<div align="center">

![B2B](https://img.shields.io/badge/🏢%20B2B-Business%20to%20Business-0B5CA8?style=for-the-badge) ➕ ![B2G](https://img.shields.io/badge/🏛️%20B2G-Business%20to%20Government-F97316?style=for-the-badge)

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 🔭 Future Scope

```mermaid
flowchart LR
    A["Advancements on the<br/>hazards detection system"] --> B["Implementation of<br/>MESH network"] --> C["Implementation of swarm<br/>system for better SAR"] --> D["Better algorithm for<br/>human detection and priority"]
    style A fill:#7FB8E8,stroke:#000,color:#000
    style B fill:#9BE05A,stroke:#000,color:#000
    style C fill:#FFA24D,stroke:#000,color:#000
    style D fill:#FFE06B,stroke:#000,color:#000
```

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 📚 References

<details>
<summary><b>Click to view references</b></summary>

<br/>

- Sharing my experience from a country where DJI is banned
- Drones in Flood Rescue Search and Emergency operations
- Applications of drone in disaster management: A scoping review
- Drone Applications for Supporting Disaster Management (DOI: `10.4236/wjet.2015.33C047`)
- Sensors and tracking methods used in wireless sensor network based unmanned search and rescue system: a review
- Optimization cost of helicopter and UAV coordinated SAR time
- How Drones Cut Costs by 95% Per Hour in Disaster Management

<!-- TODO: add the actual URLs for each reference from your PDF links -->

</details>

<img src="https://capsule-render.vercel.app/api?type=rect&color=gradient&customColorList=12,20,24&height=3" width="100%" alt=""/>

## 👥 Meet the Team

<div align="center">

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=26&pause=1200&color=38BDF8&center=true&vCenter=true&width=700&lines=THE+MINDS+BEHIND+DRONE+ACHARYAA;Team+MUYA+CHAKHWI;Building+tech+that+saves+lives" alt="Team typing animation" />

<br/><br/>

<table>
<tr>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Sania+Debbarma&background=F97316&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Sania Debbarma"/><br/>
<b>Sania Debbarma</b><br/>
👑 <i>Team Leader</i><br/>
<a href="https://www.linkedin.com/in/YOUR-SANIA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Debashis+Deb&background=0B5CA8&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Debashis Deb"/><br/>
<b>Debashis Deb</b><br/>
<a href="https://www.linkedin.com/in/YOUR-DEBASHIS-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Anup+Sarkar&background=16A34A&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Anup Sarkar"/><br/>
<b>Anup Sarkar</b><br/>
<a href="https://www.linkedin.com/in/YOUR-ANUP-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
</tr>
<tr>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Gourab+Das&background=9333EA&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Gourab Das"/><br/>
<b>Gourab Das</b><br/>
<a href="https://www.linkedin.com/in/YOUR-GOURAB-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Diya+Das&background=DB2777&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Diya Das"/><br/>
<b>Diya Das</b><br/>
<a href="https://www.linkedin.com/in/YOUR-DIYA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
<td align="center" width="200">
<img src="https://ui-avatars.com/api/?name=Samadrita+Chakraborty&background=0891B2&color=ffffff&size=200&bold=true&rounded=true" width="130" alt="Samadrita Chakraborty"/><br/>
<b>Samadrita Chakraborty</b><br/>
<a href="https://www.linkedin.com/in/YOUR-SAMADRITA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a>
</td>
</tr>
</table>

<!--
  OPTIONAL: to show real photos, replace an avatar's src with the person's GitHub photo:
  https://github.com/THEIR-GITHUB-USERNAME.png
  OPTIONAL: to add a team group photo, upload it to the repo and add under the typing animation:
  <img src="team.jpg" width="70%" alt="Team Muya Chakhwi"/>
-->

</div>

<br/>

<div align="center">

**Smart India Hackathon 2026 · Problem Statement 26177 · Team ID 181771**

*Drone Acharyaa: Intelligence That Guides, Eyes That Save* 🚁

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=160&section=footer&text=Eyes%20That%20Save&fontSize=34&fontColor=ffffff&animation=twinkling&fontAlignY=65" width="100%" alt="footer"/>

</div>
