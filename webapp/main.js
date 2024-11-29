const simulationMenuItems = [
    {
        title: 'Classic Espresso',
        price: '$4.99',
        description: 'Rich, full-bodied espresso with a perfect crema layer',
        image: 'https://images.unsplash.com/photo-1514432324607-a09d9b4aefdd?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'simulation'
    },
    {
        title: 'Caramel Macchiato',
        price: '$5.99',
        description: 'Smooth espresso with vanilla and caramel, topped with foam',
        image: 'https://images.unsplash.com/photo-1485808191679-5f86510681a2?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'simulation'
    },
    {
        title: 'Matcha Latte',
        price: '$6.49',
        description: 'Premium Japanese matcha blended with steamed milk',
        image: 'https://images.unsplash.com/photo-1515823662972-da6a2e4d3002?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'simulation'
    },
    {
        title: 'Vanilla Frappuccino',
        price: '$5.99',
        description: 'Creamy vanilla blend topped with whipped cream',
        image: 'https://images.unsplash.com/photo-1461023058943-07fcbe16d735?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'simulation'
    }
];


const realMenuItems = [
    {
        title: 'Iced Americano',
        price: '$4.49',
        description: 'Espresso shots topped with cold water for a light and crisp finish',
        image: 'https://images.unsplash.com/photo-1517701550927-30cf4ba1dba5?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'real'
    },
    {
        title: 'Mocha Latte',
        price: '$5.99',
        description: 'Espresso with steamed milk, chocolate syrup, and whipped cream',
        image: 'https://images.unsplash.com/photo-1572442388796-11668a67e53d?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'real'
    },
    {
        title: 'Chai Tea Latte',
        price: '$5.49',
        description: 'Black tea infused with spices and steamed milk',
        image: 'https://images.unsplash.com/photo-1557006021-6d123e1a1302?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'real'
    },
    {
        title: 'Caramel Frappe',
        price: '$6.49',
        description: 'Blended coffee with caramel syrup, topped with whipped cream',
        image: 'https://images.unsplash.com/photo-1586195831800-24f14c992cea?auto=format&fit=crop&q=80&w=400&h=300',
        orderType: 'real'
    }
];

function createMenuItem(item, container) {
    const menuItem = document.createElement('div');
    menuItem.className = 'menu-item';
    
    menuItem.innerHTML = `
        <div class="robot-type-overlay">
            <div class="robot-type-text">
                ${item.orderType === 'simulation' ? 'SIMULATION ROBOT' : 'REAL ROBOT'}
            </div>
        </div>
        <img src="${item.image}" alt="${item.title}">
        <div class="menu-item-content">
            <div class="menu-item-header">
                <span class="menu-item-title">${item.title}</span>
                <span class="menu-item-price">${item.price}</span>
            </div>
            <p>${item.description}</p>
            <button onclick="orderItem('${item.title}', '${item.orderType}')" class="btn">Order Now</button>
        </div>
    `;
    
    container.appendChild(menuItem);
    
    // Add hover event handlers
    const overlay = menuItem.querySelector('.robot-type-overlay');
    const text = menuItem.querySelector('.robot-type-text');
    let hoverTimeout;

    menuItem.addEventListener('mouseenter', () => {
        if (hoverTimeout) clearTimeout(hoverTimeout);
        overlay.classList.add('show');
        text.classList.add('show');
        hoverTimeout = setTimeout(() => {
            overlay.classList.remove('show');
            text.classList.remove('show');
        }, 1000);
    });

    menuItem.addEventListener('mouseleave', () => {
        if (hoverTimeout) clearTimeout(hoverTimeout);
        overlay.classList.remove('show');
        text.classList.remove('show');
    });
}

function populateMenu() {
    const simulationContainer = document.getElementById('simulation-menu-container');
    const realContainer = document.getElementById('real-menu-container');
    
    simulationContainer.innerHTML = '';
    realContainer.innerHTML = '';
    
    simulationMenuItems.forEach(item => createMenuItem(item, simulationContainer));
    realMenuItems.forEach(item => createMenuItem(item, realContainer));
}

// Initialize the page
document.addEventListener('DOMContentLoaded', () => {
    populateMenu();
    
    // Smooth scroll for navigation links
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            e.preventDefault();
            document.querySelector(this.getAttribute('href')).scrollIntoView({
                behavior: 'smooth'
            });
        });
    });
});