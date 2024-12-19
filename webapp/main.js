const simulationMenuItems = [
    {
        title: 'Spiced Pumpkin Latte',
        price: '$5.49',
        description: 'A warm blend of espresso, steamed milk, and spiced pumpkin flavor.',
        image: 'images/Pumpkin.jpg',
        orderType: 'simulation'
    },
    {
        title: 'Hazelnut Mocha',
        price: '$5.99',
        description: 'Rich espresso with steamed milk, chocolate, and hazelnut syrup.',
        image: 'images/Hazelnut.jpg',
        orderType: 'simulation'
    },
    {
        title: 'Irish Cream Coffee',
        price: '$5.49',
        description: 'A creamy espresso drink with a touch of Irish cream flavor.',
        image: 'images/Irish.webp',
        orderType: 'simulation'
    },
    {
        title: 'Cinnamon Dolce Latte',
        price: '$6.49',
        description: 'A comforting blend of espresso, steamed milk, and cinnamon dolce syrup, topped with whipped cream and cinnamon sugar.',
        image: 'images/Cinnamon.jpg',
        orderType: 'simulation'
    }
];

const realMenuItems = [
    {
        title: 'Iced Coconut Caramel Latte',
        price: '$5.99',
        description: 'A tropical twist with espresso, coconut milk, caramel and ice.',
        image: 'images/Coconut.webp',
        orderType: 'real'
    },
    {
        title: 'Iced Lavender Latte',
        price: '$6.49',
        description: 'Chilled espresso with lavender syrup and creamy milk over ice.',
        image: 'images/Lavender.jpg',
        orderType: 'real'
    },
    {
        title: 'Iced Mocha Mint',
        price: '$5.99',
        description: 'A refreshing espresso with mint, chocolate, milk, and ice.',
        image: 'images/Mocha.jpg',
        orderType: 'real'
    },
    {
        title: 'Iced Honey Almond Latte',
        price: '$6.49',
        description: 'Smooth espresso with honey, almond milk, and a touch of ice.',
        image: 'images/Almond.jpg',
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