#!/bin/bash

# --- KiCad Backup Script ---
# This script creates a compressed archive of your custom KiCad files.

# 1. Define backup directory and filename
BACKUP_DIR="/home/eduardo/KiCad_Backups"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BACKUP_FILE="kicad_backup_${TIMESTAMP}.tar.gz"

# 2. Define source directories to backup
# Change these paths if your projects are in a different location.
KICAD_CONFIG_DIR="$HOME/.config/kicad/9.0"
PROJECT_DIR="/home/eduardo/Documents/070_NIXIE_CLOCK/stm32-nixie-clock/hardware/kicad"

# 3. Create the backup directory if it doesn't exist
mkdir -p "$BACKUP_DIR"

# 4. Create the backup archive
echo "Creating backup of KiCad configuration and project files..."

tar -czvf "$BACKUP_DIR/$BACKUP_FILE" \
"$KICAD_CONFIG_DIR" \
"$PROJECT_DIR" \
--exclude='*.bak' \
--exclude='*~' \
--exclude='*.lck' \
--exclude='*~'

# 5. Check if the backup was successful
if [ $? -eq 0 ]; then
    echo "✅ Backup completed successfully!"
    echo "Backup file: $BACKUP_DIR/$BACKUP_FILE"
else
    echo "❌ Backup failed. Please check for errors."
    exit 1
fi

# 6. Optional: Clean up old backups (e.g., keep the last 5)
echo "Cleaning up old backups..."
ls -t "$BACKUP_DIR" | tail -n +6 | xargs -d '\n' rm --

echo "Cleanup complete."
