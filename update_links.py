#!/usr/bin/env python3
import os
import re
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

def determine_component_from_url(url, file_path):
    """Determine which component a file belongs to based on URL and file path."""
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

def determine_subdir_from_url(url):
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

def update_markdown_links():
    """Update markdown files to include links to local resources."""
    updated_files = 0
    
    for md_file in Path("research").glob("**/*.md"):
        with open(md_file, "r") as f:
            content = f.read()
        
        # Find all Waveshare links
        waveshare_links = re.findall(r'\[([^\]]+)\]\((https://files\.waveshare\.com/[^\)]+)\)', content)
        
        if not waveshare_links:
            continue
        
        # Update content with local links
        updated_content = content
        for link_text, url in waveshare_links:
            filename = url.split("/")[-1]
            
            # Determine component and subdir
            component = determine_component_from_url(url, str(md_file))
            subdir = determine_subdir_from_url(url)
            
            # Create component directory if it doesn't exist
            if component not in COMPONENT_DIRS:
                COMPONENT_DIRS[component] = f"research/hardware/misc/{component}"
                os.makedirs(f"{COMPONENT_DIRS[component]}/{subdir}", exist_ok=True)
            
            # Create relative path to the local file
            rel_path = os.path.relpath(
                f"{COMPONENT_DIRS[component]}/{subdir}/{filename}",
                os.path.dirname(md_file)
            )
            
            # Replace the link with both external and local links
            old_link = f"[{link_text}]({url})"
            new_link = f"[{link_text}]({url}) ([Local Copy]({rel_path}))"
            updated_content = updated_content.replace(old_link, new_link)
        
        # Write updated content back to file
        if updated_content != content:
            with open(md_file, "w") as f:
                f.write(updated_content)
            print(f"Updated links in {md_file}")
            updated_files += 1
    
    print(f"Updated {updated_files} files with local resource links")

if __name__ == "__main__":
    update_markdown_links()
