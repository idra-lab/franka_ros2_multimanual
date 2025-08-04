// Wait for the DOM to be fully loaded before running the script
document.addEventListener('DOMContentLoaded', () => {

    const navLinks = document.querySelectorAll('nav a');
    const contentPages = document.querySelectorAll('.page-content');

    function showPage(pageId) {
        // Hide all pages and remove active class from all links
        contentPages.forEach(page => {
            page.classList.remove('active-page');
        });
        navLinks.forEach(link => {
            link.classList.remove('active');
        });

        // Show the target page
        const pageToShow = document.getElementById(pageId);
        if (pageToShow) {
            pageToShow.classList.add('active-page');
        }

        // Highlight the active navigation link
        const activeLink = document.querySelector(`nav a[href="#${pageId}"]`);
        if (activeLink) {
            activeLink.classList.add('active');
        }
    }

    // Add click event listeners to all navigation links
    navLinks.forEach(link => {
        link.addEventListener('click', (event) => {
            event.preventDefault(); // Prevent the default jump-to-anchor behavior
            const pageId = link.getAttribute('href').substring(1); // Get page ID from href (e.g., "home")
            
            // Update the URL hash without reloading the page
            history.pushState(null, null, `#${pageId}`);

            showPage(pageId);
        });
    });

    // Handle back/forward browser navigation
    window.addEventListener('popstate', () => {
        const hash = window.location.hash.substring(1);
        const pageId = hash || 'home'; // Default to 'home' if hash is empty
        showPage(pageId);
    });

    // Show initial page based on the URL hash, or default to 'home'
    const initialHash = window.location.hash.substring(1);
    const initialPageId = initialHash || 'home';
    showPage(initialPageId);
});