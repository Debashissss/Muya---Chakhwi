<div align="center">

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=270&section=header&text=MUYA%20CHAKHWI&fontSize=74&fontColor=ffffff&animation=fadeIn&fontAlignY=36&desc=Smart%20India%20Hackathon%202026&descAlignY=58&descSize=22" width="100%" alt="MUYA CHAKHWI"/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=800&size=32&pause=900&color=F97316&center=true&vCenter=true&width=860&lines=%F0%9F%9A%81+DRONE+ACHARYAA;Intelligence+That+Guides%2C+Eyes+That+Save;AI-Powered+Search-and-Rescue+Drone;Offline-First+%7C+GPS-Denied+Ready+%7C+Human-Verified" alt="Typing animation"/>

<br/>

![SIH 2026](https://img.shields.io/badge/SIH-2026-0B5CA8?style=for-the-badge)
![PS](https://img.shields.io/badge/PS-26177-F97316?style=for-the-badge)
![Team](https://img.shields.io/badge/Team%20ID-181771-16A34A?style=for-the-badge)
![Theme](https://img.shields.io/badge/Robotics%20%26%20Drones-Hardware-9333EA?style=for-the-badge)

![FP](https://img.shields.io/badge/False%20Positives-%3C5%25-4ADE80?style=flat-square&labelColor=0B2545)
![Overlap](https://img.shields.io/badge/Survey%20Overlap-30%25-38BDF8?style=flat-square&labelColor=0B2545)
![Bands](https://img.shields.io/badge/Priority%20Bands-4-F97316?style=flat-square&labelColor=0B2545)
![Fix](https://img.shields.io/badge/RTK--free%20Fix-%C2%B12.5%20m-FACC15?style=flat-square&labelColor=0B2545)
![Cloud](https://img.shields.io/badge/Cloud%20Needed-0-F472B6?style=flat-square&labelColor=0B2545)

</div>

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=24&duration=2500&pause=1500&color=38BDF8&width=600&height=40&lines=THE+PROBLEM" alt="The Problem"/>

> **PS 26177:** A deployable AI-powered autonomous drone that aids search-and-rescue by detecting people and hazards, improving responder safety and cutting victim discovery time.

- 🕒 **Delayed rescue** in inaccessible or damaged terrain
- 📡 **Connectivity loss** breaks cloud-dependent tools
- ⚠️ **Rescuers at risk** during manual assessments
- 🛰️ **GPS-denied zones** make normal drones unreliable
- ⚖️ **No prioritization** among multiple survivors

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=24&duration=2500&pause=1500&color=F97316&width=600&height=40&lines=OUR+SOLUTION%3A+DRONE+ACHARYAA" alt="Our Solution"/>

A hexacopter with **edge AI** that sees with **RGB + thermal**, finds survivors and hazards, ranks them by urgency and guides rescuers, **all without internet**.

<div align="center">

![Sense](https://img.shields.io/badge/-SENSE-0EA5E9?style=for-the-badge) ➜ ![Detect](https://img.shields.io/badge/-DETECT-22C55E?style=for-the-badge) ➜ ![Prioritize](https://img.shields.io/badge/-PRIORITIZE-F97316?style=for-the-badge) ➜ ![Map](https://img.shields.io/badge/-MAP-EAB308?style=for-the-badge) ➜ ![Plan](https://img.shields.io/badge/-PLAN-6366F1?style=for-the-badge) ➜ ![Guide](https://img.shields.io/badge/-GUIDE-EC4899?style=for-the-badge) ➜ ![Alert](https://img.shields.io/badge/-ALERT-EF4444?style=for-the-badge)

</div>

| 📡 Link | What it carries | Status in a disaster zone |
|---|---|---|
| **LoRa** | Critical geo-tagged alerts, drone → command center | ✅ Always works |
| **Wi-Fi / 4G** | Video + images, buffered in local SQLite | ⏳ Auto-sync on reconnect |
| **Internet / cloud** | Not required | ❌ Not needed |

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=24&duration=2500&pause=1500&color=4ADE80&width=600&height=40&lines=WHAT+MAKES+IT+DIFFERENT" alt="What makes it different"/>

- 🐝 Swarm-based collaborative search
- 📴 Offline AI, reliable with zero connectivity
- 🌡️ RGB + Thermal + GPS + IMU fusion
- 🛣️ Dynamic safe corridors for rescuers
- 🔋 Energy-aware routes with battery kept for return
- 🌊🔥 Disaster-specific AI: floods, fire, earthquake, debris
- 💰 Lower cost per flight hour: **₹4,000 to ₹8,000** vs ₹5,000 to ₹15,000 for existing SAR drones

**Flight modes:** 📍 GPS survey grid (30% overlap, geofence) &nbsp;·&nbsp; 📴 GPS-denied (optical flow + EKF switching) &nbsp;·&nbsp; 🌲 Obstacle stop, pilot override always live

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=24&duration=2500&pause=1500&color=FACC15&width=600&height=40&lines=HOW+IT+WORKS" alt="How it works"/>

| 🔬 Method | What we do |
|---|---|
| **Edge AI detection** | YOLO11 on RGB + thermal, on a Raspberry Pi 5, fully offline |
| **Thermal fusion** | Thermal confirms RGB to filter fire and hot debris, target **<5%** false positives |
| **Locate and prioritize** | Pixel to GPS, duplicate merge, hazard check, **4-band** priority queue |
| **Navigation** | Waypoint grid, energy-aware return, GPS-denied failover |
| **Trust** | Every alert shows confidence + evidence, a **human verifies** before action |

<div align="center">

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![FastAPI](https://img.shields.io/badge/FastAPI-009688?style=for-the-badge&logo=fastapi&logoColor=white)
![MAVLink](https://img.shields.io/badge/MAVLink-E4572E?style=for-the-badge)
![Leaflet](https://img.shields.io/badge/Leaflet-199900?style=for-the-badge&logo=leaflet&logoColor=white)
![SQLite](https://img.shields.io/badge/SQLite-003B57?style=for-the-badge&logo=sqlite&logoColor=white)
![RPi5](https://img.shields.io/badge/Raspberry%20Pi%205-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)
![Pixhawk](https://img.shields.io/badge/Pixhawk-6X%20v2-1F2937?style=for-the-badge)

</div>

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=24&duration=2500&pause=1500&color=F472B6&width=600&height=40&lines=IMPACT+%26+FUTURE" alt="Impact and future"/>

- 💼 **Economic:** lower search cost, smarter responder deployment, scalable fleets
- 🇮🇳 **National:** offline-first response, multi-agency coordination, less reliance on imported drones
- 🤝 **Social:** faster survivor identification, night and remote-area search, reduced responder exposure
- 💡 **Business model:** B2B + B2G
- 🔭 **Next:** better hazard detection · MESH network · swarm SAR · smarter human detection and priority

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=70&section=header" width="100%" alt=""/>

<div align="center">

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=700&size=26&pause=1200&color=38BDF8&center=true&vCenter=true&width=700&lines=MEET+TEAM+MUYA+CHAKHWI;Building+tech+that+saves+lives" alt="Meet the team"/>

<br/><br/>

<table>
<tr>
<td align="center" width="190"><img src="https://media.licdn.com/dms/image/v2/D4D03AQFt5r3GStxKgw/profile-displayphoto-crop_800_800/B4DZ9mq.1tKYAw-/0/1784133975320?e=1792022400&amp;v=beta&amp;t=8N0ie6G6EGOl03bRjuLq0cNHvM2l1tpKx76E7GHTC_0" width="120" alt="Sania Debbarma"/><br/><b>Sania Debbarma</b><br/>👑 <i>Team Leader</i><br/><a href="https://www.linkedin.com/in/YOUR-SANIA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
<td align="center" width="190"><img src="https://media.licdn.com/dms/image/v2/D5603AQH7aA9q3wHlfA/profile-displayphoto-crop_800_800/B56ZkVNYWFHUAs-/0/1756997466696?e=1792022400&amp;v=beta&amp;t=EGvziPbEFzA3IDXZgx5qvTWJYzdvHIGdZubqtIVDyPw" width="120" alt="Debashis Deb"/><br/><b>Debashis Deb</b><br/><a href="https://www.linkedin.com/in/YOUR-DEBASHIS-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
<td align="center" width="190"><img src="https://media.licdn.com/dms/image/v2/D4D03AQGjRHRPJQjDgg/profile-displayphoto-scale_400_400/B4DaAa4gopKwAg-/0/1787157417318?e=1792022400&amp;v=beta&amp;t=J0Egap5pb8Us5zV-wlGoFcydkw8jMjFXKm7cZQLA2Rs" width="120" alt="Anup Sarkar"/><br/><b>Anup Sarkar</b><br/><a href="https://www.linkedin.com/in/YOUR-ANUP-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
</tr>
<tr>
<td align="center" width="190"><img src="https://media.licdn.com/dms/image/v2/D5603AQEPsKU6Ki5s3g/profile-displayphoto-scale_400_400/B56Z6I7d95JMAg-/0/1780413752983?e=1792022400&amp;v=beta&amp;t=8kbYvHu3ssoLcwVUnIucHxdHIC1inhq_lRBFwwutEyk" width="120" alt="Gourab Das"/><br/><b>Gourab Das</b><br/><a href="https://www.linkedin.com/in/YOUR-GOURAB-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
<td align="center" width="190"><img src="https://media.licdn.com/dms/image/v2/D4D03AQGyzBMxgkSeVw/profile-displayphoto-crop_800_800/B4DZ8jAYjLJsAc-/0/1782998731822?e=1792022400&amp;v=beta&amp;t=0HI3QBAXfsgnVYQqC8988P5SC2uwsctKLmoDw6SUg4w" width="120" alt="Diya Das"/><br/><b>Diya Das</b><br/><a href="https://www.linkedin.com/in/YOUR-DIYA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
<td align="center" width="190"><img src="https://ui-avatars.com/api/?name=Samadrita+Chakraborty&background=0891B2&color=ffffff&size=200&bold=true&rounded=true" width="120" alt="Samadrita Chakraborty"/><br/><b>Samadrita Chakraborty</b><br/><a href="https://www.linkedin.com/in/YOUR-SAMADRITA-LINK"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat&logo=linkedin&logoColor=white" alt="LinkedIn"/></a></td>
</tr>
</table>

<!-- Samadrita's photo: add assets/team/samadrita.png, then replace her avatar src with that path. -->

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=170&section=footer&text=Eyes%20That%20Save&fontSize=36&fontColor=ffffff&animation=twinkling&fontAlignY=65" width="100%" alt="footer"/>

</div>
