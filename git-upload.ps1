# Script upload code lên GitHub
# Sửa các thông tin sau:
$GITHUB_USERNAME = "Cgminhh1102"  # ← Thay đổi
$REPO_NAME = "kltn"  # ← Thay đổi
$YOUR_NAME = "vinh"            # ← Thay đổi
$YOUR_EMAIL = "21020697@vnu.edu.vn"      # ← Thay đổi

Write-Host "=== UPLOADING TO GITHUB ===" -ForegroundColor Green

# 1. Khởi tạo Git
Write-Host "`n[1/7] Initializing Git repository..." -ForegroundColor Cyan
git init

# 2. Cấu hình Git
Write-Host "[2/7] Configuring Git..." -ForegroundColor Cyan
git config user.name "$YOUR_NAME"
git config user.email "$YOUR_EMAIL"

# 3. Tạo .gitignore
Write-Host "[3/7] Creating .gitignore..." -ForegroundColor Cyan
@"
# Build artifacts
build/
.cache/

# Zephyr
.west/
bootloader/
modules/
tools/
zephyr/

# IDE
.vscode/
*.code-workspace

# OS
.DS_Store
Thumbs.db
"@ | Out-File -FilePath .gitignore -Encoding utf8

# 4. Add files
Write-Host "[4/7] Adding files..." -ForegroundColor Cyan
git add .

# 5. Commit
Write-Host "[5/7] Creating commit..." -ForegroundColor Cyan
git commit -m "Initial commit: Bluetooth Mesh DSDV routing with metrics

Features:
- DSDV routing protocol (HELLO + UPDATE packets)
- Network metrics collection (RSSI, battery, congestion)
- Convergence time measurement
- LED indication for provisioning and packet receive
- Multi-hop routing with path vector
- Relay metrics collection
- Shell commands for network monitoring
"

# 6. Add remote
Write-Host "[6/7] Adding remote repository..." -ForegroundColor Cyan
git remote add origin "https://github.com/$GITHUB_USERNAME/$REPO_NAME.git"

# 7. Push
Write-Host "[7/7] Pushing to GitHub..." -ForegroundColor Cyan
git branch -M main
git push -u origin main

Write-Host "`n=== UPLOAD COMPLETE ===" -ForegroundColor Green
Write-Host "Repository: https://github.com/$GITHUB_USERNAME/$REPO_NAME" -ForegroundColor Yellow
