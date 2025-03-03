#!/usr/bin/env python3
import os
import re
import requests
import subprocess
import sys
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
    "CP210x": "CP2102",
    "ST3215": "ST3215",
    "ST3235": "ST3235",
    "QMI8658": "QMI8658",
    "AK09918": "AK09918C",
    "INA219": "INA219",
    "TB6612": "TB6612FNG",
    "ESP32": "ESP32",
    "IPEX": "IPEX_Gen_1",
    "General_Driver": "ESP32",
}

# Direct resource URLs for key components
DIRECT_RESOURCES = [
    # CP2102 Resources
    {
        "url": "https://www.silabs.com/documents/public/software/CP210x_Universal_Windows_Driver.zip",
        "component": "CP2102",
        "subdir": "drivers",
        "filename": "CP210x_Universal_Windows_Driver.zip"
    },
    {
        "url": "https://www.silabs.com/documents/public/data-sheets/CP2102-9.pdf",
        "component": "CP2102",
        "subdir": "datasheets",
        "filename": "CP2102-9.pdf"
    },
    # ST3235 Resources
    {
        "url": "https://www.waveshare.com/w/upload/5/5e/ST3235_Servo_User_Manual.pdf",
        "component": "ST3235",
        "subdir": "datasheets",
        "filename": "ST3235_Servo_User_Manual.pdf"
    },
    {
        "url": "https://www.waveshare.com/w/upload/a/a7/ST_Servo_Communication_Protocol.pdf",
        "component": "ST3235",
        "subdir": "datasheets",
        "filename": "ST_Servo_Communication_Protocol.pdf"
    },
    # ESP32 Resources
    {
        "url": "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf",
        "component": "ESP32",
        "subdir": "datasheets",
        "filename": "ESP32-WROOM-32_datasheet_en.pdf"
    },
    # QMI8658 Resources
    {
        "url": "https://www.qstcorp.com/upload/pdf/202303/QMI8658A_Datasheet_Rev_A.pdf",
        "component": "QMI8658",
        "subdir": "datasheets",
        "filename": "QMI8658A_Datasheet.pdf"
    },
]

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
        ["grep", "-r", "https://", "--include=*.md", "research/"],
        capture_output=True, text=True
    )
    
    for line in result.stdout.splitlines():
        # Extract URLs from the line
        urls = re.findall(r'https://[^\s\)\"\']+', line)
        file_path = line.split(":")[0]
        
        for url in urls:
            # Filter for relevant resource URLs
            if any(ext in url.lower() for ext in [".zip", ".pdf", ".exe", ".dmg", ".tar.gz", ".stl", ".step", ".stp"]):
                links.append((file_path, url))
    
    return links

def download_file(url, save_path):
    """Download a file from a URL and save it to the specified path."""
    try:
        print(f"Downloading {url} to {save_path}...")
        response = requests.get(url, stream=True, timeout=30)
        response.raise_for_status()
        
        with open(save_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        
        print(f"Successfully downloaded to {save_path}")
        return True
    except Exception as e:
        print(f"Error downloading {url}: {e}")
        return False

def determine_component(url, file_path):
    """Determine which component a file belongs to."""
    filename = url.split("/")[-1].lower()
    
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

def download_direct_resources():
    """Download resources directly from the DIRECT_RESOURCES list."""
    success_count = 0
    for resource in DIRECT_RESOURCES:
        component = resource["component"]
        subdir = resource["subdir"]
        url = resource["url"]
        filename = resource["filename"]
        
        # Create component directory if it doesn't exist
        if component not in COMPONENT_DIRS:
            COMPONENT_DIRS[component] = f"research/hardware/misc/{component}"
        
        component_dir = COMPONENT_DIRS[component]
        os.makedirs(f"{component_dir}/{subdir}", exist_ok=True)
        
        # Download file
        save_path = f"{component_dir}/{subdir}/{filename}"
        if download_file(url, save_path):
            success_count += 1
    
    print(f"Downloaded {success_count} out of {len(DIRECT_RESOURCES)} direct resources")
    return success_count

def main():
    # Create component directories
    create_component_dirs()
    
    # Download direct resources
    direct_success = download_direct_resources()
    
    # Find external links
    links = find_external_links()
    print(f"Found {len(links)} external links")
    
    # Download files from external links
    link_success = 0
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
        
        # Skip if file already exists
        if os.path.exists(save_path):
            print(f"File already exists: {save_path}")
            link_success += 1
            continue
        
        if download_file(url, save_path):
            link_success += 1
        
        print(f"Processed {url} for component {component} in {subdir}")
    
    print(f"Downloaded {link_success} out of {len(links)} files from external links")
    print(f"Total downloads: {direct_success + link_success} out of {len(DIRECT_RESOURCES) + len(links)}")
    
    return 0 if direct_success > 0 else 1

if __name__ == "__main__":
    sys.exit(main())
