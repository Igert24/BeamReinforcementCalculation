# 🏗️ Beam Reinforcement Calculation Algorithm (Eurocode 2)

This project implements a fully automated **reinforced concrete beam design under flexural load** algorithm according to **Eurocode 2 (EN 1992-1-1)** with parameters from the **Finnish National Annex**. It evaluates both **Ultimate Limit State (ULS)** and **Serviceability Limit State (SLS)** performance, including:

- ULS bending and shear checks  
- SLS crack width calculation (Direct method, EC2 Clause 7.3.4)  
- Minimum reinforcement verification  
- Span-to-depth deflection control  

---

## 🚀 Features

- 🔁 Iterates over multiple top/bottom rebar combinations and stirrup configs  
- ⚡ Up to 98% faster than manual design workflows  
- 📐 Crack width and deflection checks per EC2 7.3.2–7.4.2  
- ✅ Complies with SFS-EN 1992-1-1 (Finland)  

---

## 🛠️ How to Build & Run

### ✅ Requirements:
- Visual Studio 2022 or newer  
- .NET Framework (any compatible version)  

### 🔧 Steps:
1. Clone the repository or download as ZIP  
2. Open `Reinforcement Calculation.sln` in Visual Studio  
3. Build the solution (`Ctrl+Shift+B`)  
4. Run the program (`F5` or `Ctrl+F5`)  
5. View results in the console — ULS & SLS outputs are printed per combination

---

## 📂 Project Structure

```bash
📁 Beam-Reinforcement-Calculation-Algorithm
├── Program.cs                  # Main algorithm (ULS/SLS loop and output)
├── Beam-Reinforcement-Calculation-Algorithm.csproj
├── Reinforcement Calculation.sln
├── App.config                  # Optional: environment or runtime settings
├── README.md
└── .gitignore
