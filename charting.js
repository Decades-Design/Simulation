// Wait until the entire webpage is loaded before attaching the event listener
document.addEventListener('DOMContentLoaded', (event) => {
    // Get the download button from the HTML
    const downloadButton = document.getElementById('downloadBtn');

    // If the button doesn't exist, stop the script to avoid errors
    if (!downloadButton) {
        console.error("Download button with id 'downloadBtn' not found.");
        return;
    }

    // Add a click event listener to the button
    downloadButton.addEventListener('click', () => {
        // 1. Check if there is data to download from the simulation
        if (typeof aircraftList === 'undefined' || aircraftList.length === 0 || !aircraftList[0].history || aircraftList[0].history.time.length === 0) {
            alert("No simulation data has been recorded yet.");
            return;
        }

        const aircraft = aircraftList[0];
        const history = aircraft.history;
        const headers = Object.keys(history); // Gets ["time", "altitude", etc.]
        
        // 2. Format the data into a single CSV-formatted string
        // Start with the header row
        let csvContent = headers.join(',') + '\n';
        
        // Loop through all recorded data points and add each one as a new row
        const numRows = history.time.length;
        for (let i = 0; i < numRows; i++) {
            let row = headers.map(header => history[header][i]);
            csvContent += row.join(',') + '\n';
        }

        // 3. Create a Blob and trigger the browser download
        // A Blob is a file-like object that holds the raw data
        const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
        
        // Create a temporary, invisible link element
        const link = document.createElement('a');
        const url = URL.createObjectURL(blob); // Create a temporary URL for the Blob
        link.setAttribute('href', url);
        link.setAttribute('download', 'simulation_data.csv'); // Set the desired filename
        
        // Add the link to the page, programmatically click it, and then remove it
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
    });
});