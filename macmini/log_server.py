#!/usr/bin/env python3

import http.server
import socketserver
import os
import html
from pathlib import Path

PORT = 8100
DIRECTORY = "/Users/frc6328/robot/logs"


def human_readable_size(size, decimal_places=2):
    """Converts a file size in bytes to a human-readable format (KB, MB, etc.)."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024.0:
            break
        size /= 1024.0
    return f"{size:.{decimal_places}f} {unit}"


class CustomRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def list_directory(self, path):
        """
        Overrides the default directory listing to create a custom index page.
        This method is called when a directory is requested.
        """
        try:
            # Get a list of all items in the directory
            items = os.listdir(path)
        except OSError:
            self.send_error(404, "No permission to list directory")
            return None

        # Calculate the total size of all files in the directory
        total_size_bytes = sum(
            os.path.getsize(os.path.join(path, name))
            for name in items
            if os.path.isfile(os.path.join(path, name))
        )
        readable_total_size = human_readable_size(total_size_bytes)

        # Sort items in reverse alphabetical order
        items.sort(key=str.lower, reverse=True)

        # Move randomized and Hoot logs to the end of the list
        def sort_index(name: str):
            if name.endswith(".hoot"):
                return 2
            elif name.startswith("akit_") and not "-" in name:
                return 1
            else:
                return 0
        items.sort(key=sort_index)

        # Start building the HTML response
        html_content = [
            '<!DOCTYPE html>',
            '<html>',
            '<head>',
            '<title>🍎 Idun Robot Logs</title>',
            '<meta charset="utf-8">',
            '<style>',
            '  body { font-family: sans-serif; padding: 2em; }',
            '  ul { list-style-type: none; padding: 0; }',
            '  li { margin-bottom: 0.5em; }',
            '  a { text-decoration: none; color: #0066cc; }',
            '  a:hover { text-decoration: underline; }',
            '  .size { color: #666; margin-left: 1em; }',
            '</style>',
            '</head>',
            '<body>',
            f'<h1>🍎 Idun Robot Logs ({readable_total_size})</h1><hr>',
            '<ul>'
        ]

        # Add each file to the list
        for name in items:
            full_path = os.path.join(path, name)
            if os.path.isfile(full_path):
                file_size = os.path.getsize(full_path)
                html_content.append(
                    f'<li>'
                    f'<a href="{html.escape(name)}" download>{html.escape(name)}</a>'
                    f'<span class="size">{human_readable_size(file_size)}</span>'
                    f'</li>'
                )

        html_content.extend(['</ul><hr></body>', '</html>'])

        # Finalize and send the response
        encoded_html = '\n'.join(html_content).encode('utf-8')
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded_html)))
        self.end_headers()
        self.wfile.write(encoded_html)


# --- Main Execution ---
if __name__ == "__main__":
    # Ensure the target directory exists
    Path(DIRECTORY).mkdir(exist_ok=True)

    # Create a TCP server
    with socketserver.ThreadingTCPServer(("0.0.0.0", PORT), CustomRequestHandler) as httpd:
        print(f"✅ Server started successfully!")
        print(f"📂 Serving logs from the '{DIRECTORY}' directory.")
        print(f"🔗 Access the server at port {PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server...")
            httpd.shutdown()
