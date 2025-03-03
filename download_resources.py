#!/usr/bin/env python3
import os
import re
import requests
import subprocess
from pathlib import Path

# Map of component names to their directory paths
COMPONENT_DIRS = {
    "CP2102": "research/hardware/communication/CP2102",
    "ST3235": "research/hardware/servos/ST3235",
    "ST3215": "research/hardware/servos/ST3215",
    "QMI8658": "research/hardware/sensors/QMI8658",
    "AK09918C": "research/hardware/sensors/AK09918C",
    "INA219": "research/hardware/sensors/INA219",
    "TB6612FNG": "research/hardware/sensors/TB6612FNG",
    "ESP32": "research/hardware/main_control/ESP32-WROOM-32",
    "IPEX_Gen_1": "research/hardware/communication/IPEX_Gen_1",
}

# Map of file patterns to component names
FILE_PATTERNS = {
    "CP210x_Windows_Drivers.zip": "CP2102",
    "ST3215_Servo_User_Manual.pdf": "ST3215",
    "ST3235": "ST3235",
    "QMI8658A.pdf": "QMI8658",
    "AK09918C": "AK09918C",
    "INA219": "INA219",
    "TB6612FNG": "TB6612FNG",
    "ESP32": "ESP32",
    "IPEX": "IPEX_Gen_1",
    "General_Driver_for_Robots": "ESP32",
}

def create_component_dirs():
    """Create component directories if they don't exist."""
    for component, dir_path in COMPONENT_DIRS.items():
        # Create main component directory
        os.makedirs(dir_path, exist_ok=True)
        
        # Create subdirectories
        os.makedirs(f"{dir_path}/drivers", exist_ok=True)
        os.makedirs(f"{dir_path}/datasheets", exist_ok=True)
        
        # Create 3D models directory for servos
        if component in ["ST3235", "ST3215"]:
            os.makedirs(f"{dir_path}/3d_models", exist_ok=True)
        
        # Create README if it doesn't exist
        readme_path = f"{dir_path}/README.md"
        if not os.path.exists(readme_path):
            with open(readme_path, "w") as f:
                f.write(f"# {component} Resources\n\n")
                f.write("This directory contains local copies of resources for the ")
                f.write(f"{component} component used in the RoArm-M3 Pro.\n\n")
                f.write("## Contents\n\n")
                f.write("- `drivers/`: Driver software\n")
                f.write("- `datasheets/`: Technical documentation\n")
                if component in ["ST3235", "ST3215"]:
                    f.write("- `3d_models/`: 3D model files\n")
                f.write("\n## Download Date\n\n")
                f.write("These resources were downloaded on March 3, 2025.\n")

def find_external_links():
    """Find all external links in markdown files."""
    links = []
    result = subprocess.run(
        ["grep", "-r", "https://files.waveshare.com", "--include=*.md", "research/"],
        capture_output=True, text=True
    )
    
    for line in result.stdout.splitlines():
        # Extract URLs from the line
        urls = re.findall(r'https://files\.waveshare\.com/[^\s\)\"\']+', line)
        file_path = line.split(":")[0]
        
        for url in urls:
            links.append((file_path, url))
    
    return links

def download_file(url, save_path):
    """Download a file from a URL and save it to the specified path."""
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        with open(save_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        
        print(f"Downloaded {url} to {save_path}")
        return True
    except Exception as e:
        print(f"Error downloading {url}: {e}")
        return False

def determine_component(url, file_path):
    """Determine which component a file belongs to."""
    filename = url.split("/")[-1]
    
    # Check if filename matches any patterns
    for pattern, component in FILE_PATTERNS.items():
        if pattern.lower() in filename.lower():
            return component
    
    # If no match, try to determine from the file path
    for component in COMPONENT_DIRS.keys():
        if component.lower() in file_path.lower():
            return component
    
    # Default to a general directory
    return "misc"

def determine_subdir(url):
    """Determine which subdirectory a file should go in based on its extension."""
    filename = url.split("/")[-1].lower()
    
    if filename.endswith((".zip", ".exe", ".dmg", ".tar.gz")):
        return "drivers"
    elif filename.endswith((".pdf", ".txt", ".doc", ".docx")):
        return "datasheets"
    elif any(ext in filename for ext in [".step", ".stp", ".stl", ".f3d", ".3d", "3d", "model"]):
        return "3d_models"
    else:
        return "misc"

def main():
    # Create component directories
    create_component_dirs()
    
    # Find external links
    links = find_external_links()
    print(f"Found {len(links)} external links")
    
    # Download files
    for file_path, url in links:
        component = determine_component(url, file_path)
        subdir = determine_subdir(url)
        
        # Create component directory if it doesn't exist
        if component not in COMPONENT_DIRS:
            COMPONENT_DIRS[component] = f"research/hardware/misc/{component}"
            os.makedirs(f"{COMPONENT_DIRS[component]}", exist_ok=True)
        
        component_dir = COMPONENT_DIRS[component]
        os.makedirs(f"{component_dir}/{subdir}", exist_ok=True)
        
        # Download file
        filename = url.split("/")[-1]
        save_path = f"{component_dir}/{subdir}/{filename}"
        download_file(url, save_path)
        
        print(f"Processed {url} for component {component} in {subdir}")

if __name__ == "__main__":
    main()
