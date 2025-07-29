"""
SSD1306 Logo Converter
Author: Mohammad Shakeri 
Version: 1.0.0
Copyright 2025. All rights reserved.
"""

import sys
import os
from PIL import Image, ImageOps
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFileDialog, QTextEdit, 
                             QMessageBox, QSpinBox, QGroupBox, QCheckBox, QComboBox,
                             QRadioButton, QButtonGroup, QFrame)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QFont

class SSD1306LogoConverter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SSD1306 Logo Converter")
        self.setGeometry(100, 100, 900, 700)
        self.original_image = None
        self.processed_image = None
        
        self.initUI()
        
    def initUI(self):
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        main_widget.setLayout(layout)
        
        # Title
        title = QLabel("SSD1306 OLED Logo Converter")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Input file selection
        file_group = QGroupBox("Input File")
        file_layout = QVBoxLayout()
        
        file_select_layout = QHBoxLayout()
        self.file_path = QLineEdit()
        self.file_path.setPlaceholderText("Select an image file (BMP, PNG, JPG, etc.)...")
        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_file)
        
        file_select_layout.addWidget(self.file_path)
        file_select_layout.addWidget(self.browse_button)
        file_layout.addLayout(file_select_layout)
        
        # Image preview (placeholder)
        self.image_preview = QLabel("No image selected")
        self.image_preview.setMinimumHeight(100)
        self.image_preview.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0;")
        self.image_preview.setAlignment(Qt.AlignCenter)
        file_layout.addWidget(self.image_preview)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Configuration options
        config_layout = QHBoxLayout()
        
        # Array name and size settings
        settings_group = QGroupBox("Array Settings")
        settings_layout = QVBoxLayout()
        
        # Array name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Array Name:"))
        self.array_name = QLineEdit()
        self.array_name.setText("my_logo")
        name_layout.addWidget(self.array_name)
        settings_layout.addLayout(name_layout)
        
        # Size controls
        size_layout = QHBoxLayout()
        size_layout.addWidget(QLabel("Width:"))
        self.width_spin = QSpinBox()
        self.width_spin.setRange(1, 128)
        self.width_spin.setValue(32)
        self.width_spin.valueChanged.connect(self.update_height_from_width)
        size_layout.addWidget(self.width_spin)
        
        size_layout.addWidget(QLabel("Height:"))
        self.height_spin = QSpinBox()
        self.height_spin.setRange(1, 64)
        self.height_spin.setValue(32)
        self.height_spin.valueChanged.connect(self.update_width_from_height)
        size_layout.addWidget(self.height_spin)
        settings_layout.addLayout(size_layout)
        
        # Keep aspect ratio
        self.aspect_check = QCheckBox("Maintain aspect ratio")
        self.aspect_check.setChecked(True)
        settings_layout.addWidget(self.aspect_check)
        
        settings_group.setLayout(settings_layout)
        config_layout.addWidget(settings_group)
        
        # Output format options
        format_group = QGroupBox("Output Format")
        format_layout = QVBoxLayout()
        
        # Bit organization
        self.bit_group = QButtonGroup()
        self.msb_radio = QRadioButton("MSB First (bit 7 = leftmost)")
        self.lsb_radio = QRadioButton("LSB First (bit 0 = leftmost)")
        self.msb_radio.setChecked(True)
        self.bit_group.addButton(self.msb_radio)
        self.bit_group.addButton(self.lsb_radio)
        format_layout.addWidget(self.msb_radio)
        format_layout.addWidget(self.lsb_radio)
        
        # Add separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        format_layout.addWidget(line)
        
        # Invert colors
        self.invert_check = QCheckBox("Invert colors (white becomes black)")
        format_layout.addWidget(self.invert_check)
        
        # Bytes per line
        bytes_layout = QHBoxLayout()
        bytes_layout.addWidget(QLabel("Bytes per line:"))
        self.bytes_per_line = QSpinBox()
        self.bytes_per_line.setRange(1, 32)
        self.bytes_per_line.setValue(4)
        bytes_layout.addWidget(self.bytes_per_line)
        bytes_layout.addStretch()
        format_layout.addLayout(bytes_layout)
        
        format_group.setLayout(format_layout)
        config_layout.addWidget(format_group)
        
        layout.addLayout(config_layout)
        
        # Convert button
        self.convert_button = QPushButton("Convert to C Array")
        self.convert_button.clicked.connect(self.convert_image)
        self.convert_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }")
        layout.addWidget(self.convert_button)
        
        # Output display
        output_group = QGroupBox("Generated C Code")
        output_layout = QVBoxLayout()
        
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setStyleSheet("font-family: 'Courier New', monospace; font-size: 10pt;")
        self.output_text.setMinimumHeight(200)
        output_layout.addWidget(self.output_text)
        
        # Save button
        button_layout = QHBoxLayout()
        self.save_button = QPushButton("Save as .h File")
        self.save_button.clicked.connect(self.save_to_file)
        self.save_button.setEnabled(False)
        self.copy_button = QPushButton("Copy to Clipboard")
        self.copy_button.clicked.connect(self.copy_to_clipboard)
        self.copy_button.setEnabled(False)
        
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.copy_button)
        button_layout.addStretch()
        output_layout.addLayout(button_layout)
        
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)

        # Info button
        info_button = QPushButton("About")
        info_button.clicked.connect(self.show_info)
        layout.addWidget(info_button)

    def show_info(self):
        info_text = (
            "SSD1306 Logo Converter\n"
            "Author: Mohammad Shakeri\n"
            "Version: 1.0.0\n"
            "Copyright 2025. All rights reserved."
        )
        QMessageBox.information(self, "About", info_text)

    def browse_file(self):
        file_name, _ = QFileDialog.getOpenFileName(
            self, "Open Image File", "", 
            "Image Files (*.bmp *.png *.jpg *.jpeg *.gif *.tiff);;All Files (*)"
        )
        if file_name:
            self.file_path.setText(file_name)
            try:
                self.original_image = Image.open(file_name)
                width, height = self.original_image.size
                
                # Update spinboxes with original dimensions
                self.width_spin.setValue(min(width, 128))
                self.height_spin.setValue(min(height, 64))
                
                # Update preview
                self.update_preview()
                
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Could not load image: {str(e)}")
    
    def update_preview(self):
        if self.original_image:
            # Create a preview of the image
            preview_size = (200, 150)
            preview_img = self.original_image.copy()
            preview_img.thumbnail(preview_size, Image.Resampling.LANCZOS)
            
            # Convert to QPixmap and display
            if preview_img.mode != 'RGB':
                preview_img = preview_img.convert('RGB')
            
            # Simple preview text for now
            size_text = f"Original: {self.original_image.size[0]}x{self.original_image.size[1]}"
            self.image_preview.setText(f"Image loaded\n{size_text}")
    
    def update_height_from_width(self):
        if self.aspect_check.isChecked() and self.original_image:
            original_width, original_height = self.original_image.size
            new_height = int((self.width_spin.value() / original_width) * original_height)
            self.height_spin.blockSignals(True)
            self.height_spin.setValue(min(new_height, 64))
            self.height_spin.blockSignals(False)
    
    def update_width_from_height(self):
        if self.aspect_check.isChecked() and self.original_image:
            original_width, original_height = self.original_image.size
            new_width = int((self.height_spin.value() / original_height) * original_width)
            self.width_spin.blockSignals(True)
            self.width_spin.setValue(min(new_width, 128))
            self.width_spin.blockSignals(False)
    
    def convert_image(self):
        file_path = self.file_path.text()
        array_name = self.array_name.text().strip() or "my_logo"
        
        if not file_path:
            QMessageBox.warning(self, "Warning", "Please select an image file first.")
            return
        
        try:
            # Load and process image
            img = Image.open(file_path)
            target_width = self.width_spin.value()
            target_height = self.height_spin.value()
            
            # Resize image
            if img.size != (target_width, target_height):
                img = img.resize((target_width, target_height), Image.Resampling.LANCZOS)
            
            # Convert to 1-bit black and white
            img = img.convert('1')
            
            # Invert if requested
            if self.invert_check.isChecked():
                img = ImageOps.invert(img)
            
            width, height = img.size
            pixels = img.load()
            
            # Generate C array data
            c_array_bytes = []
            bytes_per_row = (width + 7) // 8  # Round up to nearest byte
            
            for y in range(height):
                row_bytes = []
                for x_byte in range(bytes_per_row):
                    byte_value = 0
                    for bit in range(8):
                        x_pixel = x_byte * 8 + bit
                        if x_pixel < width:
                            pixel = pixels[x_pixel, y]
                            # pixel == 0 means black (ON for OLED)
                            if pixel == 0:
                                if self.msb_radio.isChecked():
                                    # MSB first: bit 7 is leftmost
                                    byte_value |= (1 << (7 - bit))
                                else:
                                    # LSB first: bit 0 is leftmost
                                    byte_value |= (1 << bit)
                    row_bytes.append(byte_value)
                c_array_bytes.extend(row_bytes)
            
            # Format the output
            self.generate_output(array_name, width, height, c_array_bytes, file_path)
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to convert image:\n{str(e)}")
    
    def generate_output(self, array_name, width, height, byte_data, source_file):
        bytes_per_line = self.bytes_per_line.value()
        
        # Generate header guard
        header_guard = f"{array_name.upper()}_H_"
        
        # Create the C code
        output_lines = []
        output_lines.append(f"#ifndef {header_guard}")
        output_lines.append(f"#define {header_guard}")
        output_lines.append("")
        output_lines.append("#include <stdint.h>")
        output_lines.append("")
        output_lines.append(f"// Auto-generated from: {os.path.basename(source_file)}")
        output_lines.append(f"// Image size: {width}x{height} pixels")
        output_lines.append(f"// Data organization: {'MSB first' if self.msb_radio.isChecked() else 'LSB first'}")
        output_lines.append(f"// Total bytes: {len(byte_data)}")
        output_lines.append("")
        output_lines.append(f"const uint8_t {array_name}_width = {width};")
        output_lines.append(f"const uint8_t {array_name}_height = {height};")
        output_lines.append("")
        output_lines.append(f"static const uint8_t {array_name}[] = {{")
        
        # Format the byte array
        for i in range(0, len(byte_data), bytes_per_line):
            line_bytes = byte_data[i:i + bytes_per_line]
            formatted_bytes = [f"0x{b:02X}" for b in line_bytes]
            
            if i + bytes_per_line < len(byte_data):
                output_lines.append(f"    {', '.join(formatted_bytes)},")
            else:
                output_lines.append(f"    {', '.join(formatted_bytes)}")
        
        output_lines.append("};")
        output_lines.append("")
        output_lines.append(f"#endif /* {header_guard} */")
        
        # Display the output
        output_text = "\n".join(output_lines)
        self.output_text.setPlainText(output_text)
        
        # Enable save and copy buttons
        self.save_button.setEnabled(True)
        self.copy_button.setEnabled(True)
        
        # Show success message with usage instructions
        usage_msg = f"""Conversion successful!

Array name: {array_name}
Dimensions: {width}x{height}
Total bytes: {len(byte_data)}

Usage in your code:
SSD1306_DrawLogo(x, y, {array_name}_width, {array_name}_height, {array_name});

Example:
SSD1306_DrawLogo(10, 10, {array_name}_width, {array_name}_height, {array_name});"""
        
        QMessageBox.information(self, "Success", usage_msg)
    
    def save_to_file(self):
        if not self.output_text.toPlainText():
            QMessageBox.warning(self, "Warning", "Nothing to save. Convert an image first.")
            return
        
        array_name = self.array_name.text().strip() or "my_logo"
        file_name, _ = QFileDialog.getSaveFileName(
            self, "Save Header File", f"{array_name}.h", "Header Files (*.h);;All Files (*)"
        )
        
        if file_name:
            try:
                with open(file_name, 'w') as f:
                    f.write(self.output_text.toPlainText())
                QMessageBox.information(self, "Success", f"File saved successfully as {file_name}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file:\n{str(e)}")
    
    def copy_to_clipboard(self):
        if not self.output_text.toPlainText():
            QMessageBox.warning(self, "Warning", "Nothing to copy. Convert an image first.")
            return
        
        clipboard = QApplication.clipboard()
        clipboard.setText(self.output_text.toPlainText())
        QMessageBox.information(self, "Success", "C code copied to clipboard!")

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    converter = SSD1306LogoConverter()
    converter.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()